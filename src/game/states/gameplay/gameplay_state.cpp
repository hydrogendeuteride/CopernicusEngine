#include "gameplay_state.h"
#include "game/states/gameplay/settings/gameplay_preload_cache.h"
#include "game/states/gameplay/orbital/orbital_physics_system.h"
#include "game/states/gameplay/orbital/orbit_runtime.h"
#include "game/states/gameplay/settings/gameplay_settings.h"
#include "game/states/gameplay/maneuver_nbody/maneuver_ui_controller.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_system.h"
#include "game/states/gameplay/prediction_nbody/prediction_system.h"
#include "game/states/gameplay/scenario/scenario_loader.h"
#include "game/component/ship_controller.h"
#include "game/states/pause_state.h"
#include "core/engine.h"
#include "core/game_api.h"
#include "core/input/input_system.h"
#include "core/orbit_plot/orbit_plot.h"
#include "core/util/logger.h"
#include "physics/physics_context.h"
#include "physics/physics_world.h"

#include <cmath>

namespace Game
{
    using detail::contact_event_type_name;

    GameplayState::GameplayState()
        : _scenario_config()
        , _prediction(std::make_unique<PredictionSystem>())
        , _kepler_prediction(std::make_unique<KeplerPredictionSystem>())
    {
    }

    GameplayState::GameplayState(ScenarioConfig scenario_config)
        : _scenario_preloaded(true)
        , _scenario_config(std::move(scenario_config))
        , _prediction(std::make_unique<PredictionSystem>())
        , _kepler_prediction(std::make_unique<KeplerPredictionSystem>())
    {
    }

    GameplayState::~GameplayState() = default;

    void GameplayState::on_enter(GameStateContext &ctx)
    {
        _world.set_api(ctx.api);
        _renderer = ctx.renderer;
        _elapsed = 0.0f;
        _fixed_time_s = 0.0;
        reset_time_warp_state();
        _kepler_maneuver.reset_session();
        _maneuver.reset_session();
        _reset_requested = false;
        _scenario_io_status.clear();
        _scenario_io_status_ok = true;
        _settings_io_status.clear();
        _settings_io_status_ok = true;
        _prediction->reset_session_state();
        _kepler_prediction->reset();

        if (ctx.renderer && ctx.renderer->_context && ctx.renderer->_context->orbit_plot)
        {
            ctx.renderer->_context->orbit_plot->clear_all();
        }

        // Load scenario from JSON if not already provided by the loading state.
        if (ctx.renderer && ctx.renderer->_assetManager)
        {
            if (!_scenario_preloaded)
            {
                const std::string scenario_path =
                        ctx.renderer->_assetManager->assetPath("scenarios/default_gameplay.json");
                if (auto loaded = load_scenario_config(scenario_path))
                {
                    _scenario_config = std::move(*loaded);
                }
                else
                {
                    Logger::error("Failed to load gameplay scenario '{}'", scenario_path);
                    return;
                }
            }

            // Auto-load gameplay settings if present.
            const std::string settings_path =
                    ctx.renderer->_assetManager->assetPath(_settings_rel_path);
            if (auto loaded_settings = load_gameplay_settings(settings_path))
            {
                apply_settings(*loaded_settings);
            }

            const std::string keybinds_path =
                    ctx.renderer->_assetManager->assetPath(_keybinds_rel_path);
            if (auto loaded_keybinds = load_keybinds(keybinds_path))
            {
                _keybinds = std::move(*loaded_keybinds);
            }
        }

        if (ctx.api && !_outline_settings_saved)
        {
            _saved_outline_settings = ctx.api->get_outline_settings();
            _outline_settings_saved = true;

            GameAPI::Engine::OutlineSettings gameplay_outline = _saved_outline_settings;
            gameplay_outline.enabled = true;
            gameplay_outline.suppressHoverWhenSelected = true;
            gameplay_outline.hover.enabled = true;
            gameplay_outline.hover.scope = GameAPI::Engine::SelectionLevel::Object;
            gameplay_outline.hover.useSelectionLevel = false;
            gameplay_outline.selection.enabled = true;
            gameplay_outline.selection.scope = GameAPI::Engine::SelectionLevel::Member;
            gameplay_outline.selection.useSelectionLevel = true;
            ctx.api->set_outline_settings(gameplay_outline);
        }

        setup_scene(ctx);
        clear_preloaded_gltf_scenes();
    }

    void GameplayState::on_exit(GameStateContext &ctx)
    {
        if (ctx.api && _outline_settings_saved)
        {
            ctx.api->set_outline_settings(_saved_outline_settings);
            _outline_settings_saved = false;
        }

        _kepler_maneuver.clear_interaction();
        _maneuver.clear_gizmo_interaction();

        _world.clear_rebase_anchor();
        _world.clear();
        _world.set_physics(nullptr);
        _world.set_physics_context(nullptr);
        _world.set_api(nullptr);
        _orbit.reset();
        _contact_log.clear();
        _prediction->reset_session_state();
        _kepler_prediction->reset();
        _renderer = nullptr;
        if (ctx.renderer && ctx.renderer->_context && ctx.renderer->_context->orbit_plot)
        {
            ctx.renderer->_context->orbit_plot->clear_all();
        }
        reset_time_warp_state();
        _kepler_maneuver.reset_session();
        _maneuver.reset_session();
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (ctx.renderer && ctx.renderer->_context)
        {
            if (ctx.renderer->_context->physics_context == _physics_context.get())
            {
                ctx.renderer->_context->physics_context = nullptr;
            }
        }
        _physics_context.reset();
        _physics.reset();
#endif
    }

    void GameplayState::on_update(GameStateContext &ctx, float dt)
    {
        if (_reset_requested)
        {
            _reset_requested = false;
            setup_scene(ctx);
            return;
        }

        _elapsed += dt;
        _frame_monitor.update(dt);

        handle_time_warp_input(ctx);

        if (ctx.input && ctx.input->key_pressed(Key::Escape))
        {
            _pending = StateTransition::push<PauseState>();
            return;
        }

        const float alpha = ctx.interpolation_alpha();
        ComponentContext comp_ctx = build_component_context(ctx, alpha);
        _world.entities().update_components(comp_ctx, dt);
        _world.entities().sync_to_render(*ctx.api, alpha);

        refresh_maneuver_node_runtime_cache(ctx);

        // Draw orbit debug using the same interpolation alpha as rendering to avoid visual offset.
        draw_prediction(ctx);
        ManeuverUiController::Context maneuver_ui = build_maneuver_ui_context(ctx);
        ManeuverUiController::emit_node_debug_overlay(maneuver_ui);
    }

    OrbitalPhysicsSystem::Context GameplayState::build_orbital_physics_context()
    {
        return GameplayOrbitalContextBuilder(GameplayOrbitalContextInputs{
                .renderer = _renderer,
                .world = _world,
                .orbit = _orbit,
                .physics = _physics.get(),
                .physics_context = _physics_context.get(),
                .scenario_config = _scenario_config,
                .spacecraft_gravity_mode =
                        spacecraft_orbit_prediction_uses_kepler()
                                ? SpacecraftGravityMode::SoiKepler
                                : SpacecraftGravityMode::NBody,
                .soi_switch_options = _kepler_prediction_options.primary_switch,
                .kepler_propagation = _kepler_prediction_options.propagation,
                .soi_kepler_max_step_s = 60.0,
                .keybinds = &_keybinds,
                .ui_capture_keyboard = [this](const GameStateContext &frame_ctx) {
                    return ui_capture_keyboard(frame_ctx);
                },
                .mark_prediction_dirty = [this]() {
                    mark_prediction_dirty();
                },
        }).build();
    }

    void GameplayState::on_fixed_update(GameStateContext &ctx, float fixed_dt)
    {
        if (_reset_requested)
        {
            _reset_requested = false;
            setup_scene(ctx);
            return;
        }

        update_maneuver_nodes_time_warp(ctx, fixed_dt);

        const TimeWarpState::Mode desired_mode = _time_warp.mode_for_level(_time_warp.warp_level);
        _time_warp.mode = desired_mode;

        const double warp_factor = _time_warp.factor();
        if (desired_mode == TimeWarpState::Mode::RailsWarp)
        {
            OrbitalPhysicsSystem::Context orbital_physics = build_orbital_physics_context();
            if (!_orbital_physics.rails_warp_active())
            {
                (void) _orbital_physics.enter_rails_warp(orbital_physics, ctx);
            }

            if (_orbital_physics.rails_warp_active())
            {
                double dt_s = static_cast<double>(fixed_dt) * warp_factor;
                if (_maneuver.runtime().warp_to_time_active)
                {
                    const double now_s = _orbit.scenario_owner() ? _orbit.scenario_owner()->sim.time_s() : _fixed_time_s;
                    const double remaining_s = _maneuver.runtime().warp_to_time_target_s - now_s;
                    if (std::isfinite(remaining_s) && remaining_s > 0.0)
                    {
                        dt_s = std::min(dt_s, remaining_s);
                    }
                }
                _fixed_time_s += dt_s;
                _orbital_physics.set_last_sim_step_dt_s(dt_s);

                _orbital_physics.rails_warp_step(orbital_physics, ctx, dt_s);
                update_maneuver_nodes_execution(ctx);
                update_prediction(ctx, static_cast<float>(dt_s));
                return;
            }
        }

        const int physics_steps =
                (desired_mode == TimeWarpState::Mode::PhysicsWarp)
                    ? std::max(1, static_cast<int>(warp_factor))
                    : 1;

        const double effective_dt_s = static_cast<double>(fixed_dt) * static_cast<double>(physics_steps);
        _orbital_physics.set_last_sim_step_dt_s(static_cast<double>(fixed_dt));

        ComponentContext comp_ctx = build_component_context(ctx);
        OrbitalPhysicsSystem::Context orbital_physics = build_orbital_physics_context();

        for (int i = 0; i < physics_steps; ++i)
        {
            _fixed_time_s += static_cast<double>(fixed_dt);
            _world.entities().fixed_update_components(comp_ctx, fixed_dt);
            _orbital_physics.update_formation_hold(orbital_physics, static_cast<double>(fixed_dt));
            _orbital_physics.step_physics(orbital_physics, ctx, fixed_dt);
        }

        update_maneuver_nodes_execution(ctx);
        update_prediction(ctx, static_cast<float>(effective_dt_s));
    }

    void GameplayState::reset_time_warp_state()
    {
        _time_warp.warp_level = 0;
        _time_warp.mode = TimeWarpState::Mode::Realtime;
        _orbital_physics.reset();
    }

    void GameplayState::handle_time_warp_input(GameStateContext &ctx)
    {
        if (!ctx.input)
        {
            return;
        }

        if (_maneuver.runtime().warp_to_time_active)
        {
            return;
        }

        if (ui_capture_keyboard(ctx))
        {
            return;
        }

        if (ctx.input->key_pressed(Key::RightBracket))
        {
            (void) cycle_player_orbiter(ctx, +1);
        }
        if (ctx.input->key_pressed(Key::LeftBracket))
        {
            (void) cycle_player_orbiter(ctx, -1);
        }

        int level = _time_warp.warp_level;
        bool changed = false;

        if (ctx.input->key_pressed(Key::Period))
        {
            level += 1;
            changed = true;
        }
        if (ctx.input->key_pressed(Key::Comma))
        {
            level -= 1;
            changed = true;
        }
        if (ctx.input->key_pressed(Key::Slash) || ctx.input->key_pressed(Key::Backspace))
        {
            level = 0;
            changed = true;
        }

        if (changed)
        {
            set_time_warp_level(ctx, level);
        }
    }

    ComponentContext GameplayState::build_component_context(GameStateContext &ctx, float alpha)
    {
        ComponentContext comp_ctx{};
        comp_ctx.world = &_world;
        comp_ctx.api = ctx.api;
        comp_ctx.input = ctx.input;
        comp_ctx.physics = _physics.get();
        comp_ctx.keybinds = &_keybinds;
        comp_ctx.ui_capture_keyboard = ui_capture_keyboard(ctx);
        comp_ctx.interpolation_alpha = alpha;
        return comp_ctx;
    }

    bool GameplayState::ui_capture_keyboard(const GameStateContext &ctx) const
    {
        return ctx.renderer && ctx.renderer->ui() && ctx.renderer->ui()->wantCaptureKeyboard();
    }

} // namespace Game
