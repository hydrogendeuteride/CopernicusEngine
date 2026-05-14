#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/orbital/orbit_runtime.h"
#include "game/states/gameplay/settings/gameplay_settings.h"
#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_gizmo.h"
#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_orbit_pick.h"
#include "game/states/gameplay/maneuver_nbody/maneuver_ui_controller.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_system.h"
#include "game/states/gameplay/prediction_nbody/gameplay_prediction_adapter.h"
#include "game/states/gameplay/prediction_nbody/prediction_host_context_builder.h"
#include "game/states/gameplay/scenario/scenario_loader.h"
#include "game/component/ship_controller.h"
#include "core/engine.h"
#include "core/game_api.h"
#include "core/input/input_system.h"
#include "core/picking/picking_system.h"
#include "core/render_viewport.h"
#include "core/util/logger.h"
#include "physics/physics_context.h"
#include "physics/physics_world.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>

namespace Game
{

    using detail::contact_event_type_name;

    namespace
    {
        const char *spacecraft_orbit_prediction_mode_label(const SpacecraftOrbitPredictionMode mode)
        {
            switch (mode)
            {
                case SpacecraftOrbitPredictionMode::Kepler:
                    return "Kepler SOI";
                case SpacecraftOrbitPredictionMode::NBody:
                    return "N Body";
            }
            return "Unknown";
        }

        std::string resolve_asset_rel_path(const GameStateContext &ctx, const std::string &rel_path)
        {
            const std::filesystem::path rel(rel_path);
            if (rel.is_absolute())
            {
                return rel.string();
            }
            if (ctx.renderer && ctx.renderer->_assetManager)
            {
                const AssetPaths &paths = ctx.renderer->_assetManager->paths();
                if (!paths.assets.empty())
                {
                    return (paths.assets / rel).string();
                }
            }
            return rel.string();
        }

        bool build_kepler_maneuver_gizmo_view_context(const GameStateContext &ctx,
                                                      KeplerManeuverGizmoViewContext &out_view)
        {
            out_view = {};
            if (!ctx.renderer || !ctx.renderer->_sceneManager)
            {
                return false;
            }

            render::RenderViewportMetrics viewport{};
            if (!render::query_render_viewport_metrics(*ctx.renderer, viewport))
            {
                return false;
            }

            const Camera &cam = ctx.renderer->_sceneManager->getMainCamera();
            out_view.camera_world = cam.position_world;
            out_view.world_to_cam = glm::transpose(glm::dmat3(cam.getRotationMatrix()));
            out_view.letterbox_rect = KeplerManeuverViewportRect{
                    .x = viewport.letterbox_rect.offset.x,
                    .y = viewport.letterbox_rect.offset.y,
                    .width = viewport.letterbox_rect.extent.width,
                    .height = viewport.letterbox_rect.extent.height,
            };
            out_view.logical_w = static_cast<double>(viewport.logical_extent.width);
            out_view.logical_h = static_cast<double>(viewport.logical_extent.height);
            if (!(out_view.logical_w > 0.0) || !(out_view.logical_h > 0.0))
            {
                return false;
            }

            const double fov_rad = glm::radians(static_cast<double>(cam.fovDegrees));
            out_view.tan_half_fov = std::tan(fov_rad * 0.5);
            out_view.aspect = out_view.logical_w / out_view.logical_h;
            if (!std::isfinite(out_view.tan_half_fov) ||
                out_view.tan_half_fov <= 1.0e-8 ||
                !std::isfinite(out_view.aspect) ||
                out_view.aspect <= 0.0)
            {
                return false;
            }

            out_view.draw_from_swap_x = viewport.draw_from_swap_x;
            out_view.draw_from_swap_y = viewport.draw_from_swap_y;
            out_view.window_from_draw_x = viewport.window_from_draw_x;
            out_view.window_from_draw_y = viewport.window_from_draw_y;
            return std::isfinite(out_view.draw_from_swap_x) &&
                   std::isfinite(out_view.draw_from_swap_y) &&
                   std::isfinite(out_view.window_from_draw_x) &&
                   std::isfinite(out_view.window_from_draw_y);
        }

        bool kepler_orbit_pick_matches_mouse_release(const GameStateContext &ctx,
                                                     const PickingSystem::PickInfo &pick,
                                                     const KeplerManeuverPlanState &plan,
                                                     const KeplerPredictionState &prediction,
                                                     const double current_time_s)
        {
            const KeplerManeuverOrbitPickInfo pick_info{
                    .valid = pick.valid,
                    .line = pick.kind == PickingSystem::PickInfo::Kind::Line,
                    .owner_name = pick.ownerName,
                    .payload = pick.line_payload,
                    .time_s = pick.time_s,
            };
            if (!ctx.input ||
                !KeplerManeuverPick::can_create_node(pick_info, plan, prediction, current_time_s))
            {
                return false;
            }

            KeplerManeuverGizmoViewContext view{};
            glm::vec2 pick_screen{0.0f, 0.0f};
            double pick_depth_m = 0.0;
            if (!build_kepler_maneuver_gizmo_view_context(ctx, view) ||
                !KeplerManeuverGizmo::project_point(view, pick.worldPos, pick_screen, pick_depth_m))
            {
                return false;
            }

            const glm::vec2 mouse_pos = ctx.input->mouse_position();
            const glm::vec2 d = mouse_pos - pick_screen;
            constexpr float kOrbitPickActivateRadiusPx = 24.0f;
            return glm::dot(d, d) <= (kOrbitPickActivateRadiusPx * kOrbitPickActivateRadiusPx);
        }

    } // anonymous namespace

    GameplaySettings GameplayState::extract_settings() const
    {
        GameplaySettings s{};
        s.prediction_draw_full_orbit = _prediction->state().draw_full_orbit;
        s.prediction_draw_future_segment = _prediction->state().draw_future_segment;
        s.prediction_draw_velocity_ray = _prediction->state().draw_velocity_ray;
        s.prediction_line_alpha_scale = _prediction->state().line_alpha_scale;
        s.prediction_line_overlay_boost = _prediction->state().line_overlay_boost;
        s.prediction_periodic_refresh_s = _prediction->state().periodic_refresh_s;
        s.prediction_thrust_refresh_s = _prediction->state().thrust_refresh_s;
        s.prediction_sampling_policy = _prediction->state().sampling_policy;
        s.maneuver_plan_horizon = _maneuver.settings().plan_horizon;
        s.maneuver_plan_windows = _maneuver.settings().plan_windows;
        s.maneuver_plan_live_preview_active = _maneuver.settings().live_preview_active;
        s.orbit_plot_budget = _prediction->budget();
        s.debug_draw_enabled = _debug_draw_enabled;
        s.runtime_orbiter_rails_enabled = _orbit.runtime_orbiter_rails_enabled();
        s.runtime_orbiter_rails_distance_m = _orbit.runtime_orbiter_rails_distance_m();
        s.contact_log_enabled = _contact_log_enabled;
        s.contact_log_print_console = _contact_log_print_console;
        return s;
    }

    void GameplayState::apply_settings(const GameplaySettings &s)
    {
        _prediction->state().draw_full_orbit = s.prediction_draw_full_orbit;
        _prediction->state().draw_future_segment = s.prediction_draw_future_segment;
        _prediction->state().draw_velocity_ray = s.prediction_draw_velocity_ray;
        _prediction->state().line_alpha_scale = s.prediction_line_alpha_scale;
        _prediction->state().line_overlay_boost = s.prediction_line_overlay_boost;
        _prediction->state().periodic_refresh_s = s.prediction_periodic_refresh_s;
        _prediction->state().thrust_refresh_s = s.prediction_thrust_refresh_s;
        _prediction->state().sampling_policy = s.prediction_sampling_policy;
        _maneuver.settings().plan_horizon = s.maneuver_plan_horizon;
        _maneuver.settings().plan_windows = s.maneuver_plan_windows;
        _maneuver.settings().live_preview_active = s.maneuver_plan_live_preview_active;
        _prediction->budget() = s.orbit_plot_budget;
        _debug_draw_enabled = s.debug_draw_enabled;
        _orbit.runtime_orbiter_rails_enabled() = s.runtime_orbiter_rails_enabled;
        _orbit.runtime_orbiter_rails_distance_m() = s.runtime_orbiter_rails_distance_m;
        _contact_log_enabled = s.contact_log_enabled;
        _contact_log_print_console = s.contact_log_print_console;
        mark_prediction_dirty();
    }

    void GameplayState::update_kepler_maneuver_orbit_pick_creation(GameStateContext &ctx)
    {
        if (!spacecraft_orbit_prediction_uses_kepler() ||
            !_kepler_prediction ||
            !ctx.input ||
            !ctx.renderer)
        {
            return;
        }

        if (_kepler_maneuver.interaction().suppress_orbit_pick_until_left_release)
        {
            if (!ctx.input->mouse_down(MouseButton::Left) ||
                ctx.input->mouse_released(MouseButton::Left))
            {
                _kepler_maneuver.interaction().suppress_orbit_pick_until_left_release = false;
            }
            return;
        }

        if (!ctx.input->mouse_released(MouseButton::Left) ||
            ImGui::GetIO().WantCaptureMouse)
        {
            return;
        }

        const KeplerManeuverInteraction::State interaction_state = _kepler_maneuver.interaction().state;
        if (interaction_state == KeplerManeuverInteraction::State::HoverHub ||
            interaction_state == KeplerManeuverInteraction::State::HoverAxis ||
            interaction_state == KeplerManeuverInteraction::State::HoverDelete ||
            interaction_state == KeplerManeuverInteraction::State::DragAxis)
        {
            return;
        }

        PickingSystem *picking = ctx.renderer->picking();
        if (!picking)
        {
            return;
        }

        const PickingSystem::PickInfo &pick = picking->last_pick();
        const KeplerManeuverOrbitPickInfo pick_info{
                .valid = pick.valid,
                .line = pick.kind == PickingSystem::PickInfo::Kind::Line,
                .owner_name = pick.ownerName,
                .payload = pick.line_payload,
                .time_s = pick.time_s,
        };
        if (!kepler_orbit_pick_matches_mouse_release(ctx,
                                                     pick,
                                                     _kepler_maneuver.plan(),
                                                     _kepler_prediction->state(),
                                                     current_sim_time_s()))
        {
            return;
        }

        (void) apply_kepler_maneuver_command(
                KeplerManeuverCommand::add_node(
                        KeplerManeuverPick::make_node(pick_info)));
    }

    void GameplayState::on_draw_ui(GameStateContext &ctx)
    {
        GameplayPredictionAdapter prediction(build_prediction_access());
        PredictionSubjectStateProvider prediction_subjects =
                PredictionHostContextBuilder(prediction.context()).make_subject_state_provider();

        if (ImGui::BeginMainMenuBar())
        {
            if (ImGui::BeginMenu("View"))
            {
                ImGui::MenuItem("Orbit HUD", nullptr, &_show_orbit_hud);
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        if (spacecraft_orbit_prediction_uses_kepler())
        {
            _show_nbody_orbit_debug = false;
            _show_maneuver_nodes_panel = false;
        }
        else
        {
            _show_kepler_orbit_debug = false;
        }

        if (_show_orbit_hud)
        {
            const ImGuiViewport *viewport = ImGui::GetMainViewport();
            ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + 10, viewport->WorkPos.y + 10));
            ImGui::SetNextWindowBgAlpha(0.4f);

            ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize;

            if (ImGui::Begin("##GameplayHUD", nullptr, flags))
            {
                // ----------------------------------------------------------------
                // Player HUD: sim time, warp, vessel, controls
                // ----------------------------------------------------------------
                const double sim_time_s = _orbit.scenario_owner() ? _orbit.scenario_owner()->sim.time_s() : _fixed_time_s;
                const int sim_hours = static_cast<int>(std::floor(sim_time_s / 3600.0));
                const int sim_minutes = static_cast<int>(std::floor(std::fmod(sim_time_s, 3600.0) / 60.0));
                const double sim_seconds = std::fmod(sim_time_s, 60.0);

                const char *warp_mode = "Realtime";
                switch (_time_warp.mode)
                {
                    case TimeWarpState::Mode::Realtime:
                        warp_mode = "Realtime";
                        break;
                    case TimeWarpState::Mode::PhysicsWarp:
                        warp_mode = "Physics";
                        break;
                    case TimeWarpState::Mode::RailsWarp:
                        warp_mode = "Rails";
                        break;
                }

                ImGui::Text("Sim: %dh %dm %.1fs", sim_hours, sim_minutes, sim_seconds);
                const char *controlled_vessel = "None";
                if (const OrbiterInfo *player_orbiter = _orbit.find_player_orbiter())
                {
                    controlled_vessel = player_orbiter->name.c_str();
                }

                ImGui::Text("Warp: x%.0f (%s)  [,][.] change  [/]/[Backspace] x1", _time_warp.factor(), warp_mode);
                ImGui::Text("Vessel: %s", controlled_vessel);
                ImGui::Text("Orbit: %s", spacecraft_orbit_prediction_mode_label(_spacecraft_orbit_prediction_mode));
                ImGui::TextUnformatted("Switch vessel: '[' previous, ']' next");
                ImGui::Text("Real: %.1f s", _elapsed);
                ImGui::Text("[ESC] Pause");

                ImGui::Separator();
                if (ImGui::CollapsingHeader("Orbit Prediction", ImGuiTreeNodeFlags_DefaultOpen))
                {
                    if (ImGui::RadioButton("Kepler SOI", spacecraft_orbit_prediction_uses_kepler()))
                    {
                        set_spacecraft_orbit_prediction_mode(SpacecraftOrbitPredictionMode::Kepler);
                    }
                    ImGui::SameLine();
                    if (ImGui::RadioButton("N Body", spacecraft_orbit_prediction_uses_nbody()))
                    {
                        set_spacecraft_orbit_prediction_mode(SpacecraftOrbitPredictionMode::NBody);
                    }

                    if (spacecraft_orbit_prediction_uses_kepler())
                    {
                        ImGui::Checkbox("Kepler Orbit Debug", &_show_kepler_orbit_debug);
                    }
                    else
                    {
                        ImGui::Checkbox("N Body Orbit Debug", &_show_nbody_orbit_debug);
                        ImGui::SameLine();
                        ImGui::Checkbox("Maneuver Nodes", &_show_maneuver_nodes_panel);
                    }
                    ImGui::Checkbox("Frame Monitor", &_show_frame_view);
                }

                prediction.rebuild_prediction_subjects();
                const PredictionTrackState *active_prediction = prediction.active_prediction_track();
                ImGui::Separator();
                if (ImGui::CollapsingHeader("Orbit Subject", ImGuiTreeNodeFlags_DefaultOpen))
                {
                    if (!active_prediction)
                    {
                        ImGui::TextUnformatted("No active prediction track.");
                    }
                    else
                    {
                        const std::string subject_label =
                                prediction.prediction_subject_label(active_prediction->key);
                        ImGui::Text("Focused subject: %s", subject_label.c_str());

                        WorldVec3 subject_pos_world{0.0, 0.0, 0.0};
                        glm::dvec3 subject_vel_world(0.0);
                        glm::vec3 subject_vel_local_f(0.0f);
                        const bool have_subject = prediction_subjects.get_subject_world_state(
                                active_prediction->key,
                                subject_pos_world,
                                subject_vel_world,
                                subject_vel_local_f);
                        if (!have_subject)
                        {
                            ImGui::TextUnformatted("Prediction subject state unavailable.");
                        }
                        else if (active_prediction->cache.identity.valid)
                        {
                            if (!active_prediction->cache.analysis.altitude_km.empty())
                            {
                                ImGui::Text("Altitude: %.0f m",
                                            static_cast<double>(
                                                    active_prediction->cache.analysis.altitude_km.front()) *
                                                    1000.0);
                            }
                            if (!active_prediction->cache.analysis.speed_kmps.empty())
                            {
                                ImGui::Text("Speed:    %.3f km/s",
                                            static_cast<double>(
                                                    active_prediction->cache.analysis.speed_kmps.front()));
                            }
                            if (!active_prediction->is_celestial)
                            {
                                ImGui::Text("Predicted Pe: %.1f km",
                                            active_prediction->cache.analysis.periapsis_alt_km);
                                if (std::isfinite(active_prediction->cache.analysis.apoapsis_alt_km))
                                {
                                    ImGui::Text("Predicted Ap: %.1f km",
                                                active_prediction->cache.analysis.apoapsis_alt_km);
                                }
                                else
                                {
                                    ImGui::TextUnformatted("Predicted Ap: escape");
                                }
                                if (active_prediction->cache.analysis.orbital_period_s > 0.0 &&
                                    std::isfinite(active_prediction->cache.analysis.orbital_period_s))
                                {
                                    ImGui::Text("Predicted Period: %.2f min",
                                                active_prediction->cache.analysis.orbital_period_s / 60.0);
                                }
                            }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
                            const EntityId player_eid = _orbit.player_entity();
                            if (_physics && _physics_context && player_eid.is_valid() &&
                                prediction.prediction_subject_is_player(active_prediction->key))
                            {
                                const glm::dvec3 v_origin_world =
                                        _physics_context->velocity_origin_world();
                                ImGui::Text("v_origin: %.1f, %.1f, %.1f m/s",
                                            v_origin_world.x,
                                            v_origin_world.y,
                                            v_origin_world.z);
                                ImGui::Text("v_local:  %.2f, %.2f, %.2f m/s",
                                            subject_vel_local_f.x,
                                            subject_vel_local_f.y,
                                            subject_vel_local_f.z);

                                const Entity *player = _world.entities().find(player_eid);
                                if (player && player->has_physics())
                                {
                                    const Physics::BodyId body_id{player->physics_body_value()};
                                    if (_physics->is_body_valid(body_id))
                                    {
                                        const glm::vec3 w_local_f =
                                                _physics->get_angular_velocity(body_id);
                                        ImGui::Text("w_local:  %.3f, %.3f, %.3f rad/s (|w|=%.3f)",
                                                    w_local_f.x,
                                                    w_local_f.y,
                                                    w_local_f.z,
                                                    glm::length(w_local_f));
                                    }
                                }
                            }
#endif
                        }
                    }
                }

#if !(defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT)
                ImGui::Separator();
                ImGui::TextUnformatted(
                    "WARNING: Built without Jolt physics (collision test requires VULKAN_ENGINE_USE_JOLT=1).");
#endif

                // ----------------------------------------------------------------
                // Scenario save/load
                // ----------------------------------------------------------------
                if (ImGui::Button("Reset scenario"))
                {
                    _reset_requested = true;
                }
                ImGui::SameLine();
                if (ImGui::Button("Replay collision"))
                {
                    _reset_requested = true;
                }

                const std::string scenario_slot_path = resolve_asset_rel_path(ctx, _scenario_slot_rel_path);

                ImGui::SameLine();
                if (ImGui::Button("Save scenario slot"))
                {
                    if (save_scenario_config(scenario_slot_path, _scenario_config))
                    {
                        _scenario_io_status = "Saved scenario: " + scenario_slot_path;
                        _scenario_io_status_ok = true;
                    }
                    else
                    {
                        _scenario_io_status = "Save failed: " + scenario_slot_path;
                        _scenario_io_status_ok = false;
                    }
                }

                ImGui::SameLine();
                if (ImGui::Button("Load scenario slot"))
                {
                    if (auto loaded = load_scenario_config(scenario_slot_path))
                    {
                        _scenario_config = std::move(*loaded);
                        _scenario_io_status = "Loaded scenario: " + scenario_slot_path;
                        _scenario_io_status_ok = true;
                        _reset_requested = true;
                    }
                    else
                    {
                        _scenario_io_status = "Load failed: " + scenario_slot_path;
                        _scenario_io_status_ok = false;
                    }
                }

                ImGui::Text("Scenario slot: %s", scenario_slot_path.c_str());
                if (!_scenario_io_status.empty())
                {
                    if (_scenario_io_status_ok)
                    {
                        ImGui::TextUnformatted(_scenario_io_status.c_str());
                    }
                    else
                    {
                        ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "%s", _scenario_io_status.c_str());
                    }
                }

                // ----------------------------------------------------------------
                // Settings save/load
                // ----------------------------------------------------------------
                const std::string settings_path = resolve_asset_rel_path(ctx, _settings_rel_path);
                if (ImGui::Button("Save settings"))
                {
                    if (save_gameplay_settings(settings_path, extract_settings()))
                    {
                        _settings_io_status = "Saved: " + settings_path;
                        _settings_io_status_ok = true;
                    }
                    else
                    {
                        _settings_io_status = "Save failed: " + settings_path;
                        _settings_io_status_ok = false;
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("Load settings"))
                {
                    if (auto loaded = load_gameplay_settings(settings_path))
                    {
                        apply_settings(*loaded);
                        if (ctx.api)
                        {
                            ctx.api->set_debug_draw_enabled(_debug_draw_enabled);
                        }
                        _settings_io_status = "Loaded: " + settings_path;
                        _settings_io_status_ok = true;
                    }
                    else
                    {
                        _settings_io_status = "Load failed: " + settings_path;
                        _settings_io_status_ok = false;
                    }
                }
                if (!_settings_io_status.empty())
                {
                    if (_settings_io_status_ok)
                    {
                        ImGui::TextUnformatted(_settings_io_status.c_str());
                    }
                    else
                    {
                        ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "%s", _settings_io_status.c_str());
                    }
                }

                // ----------------------------------------------------------------
                // Debug toggles
                // ----------------------------------------------------------------
                ImGui::Checkbox("Contact log", &_contact_log_enabled);
                ImGui::SameLine();
                ImGui::Checkbox("Print console", &_contact_log_print_console);

                if (ctx.api)
                {
                    if (ImGui::Checkbox("Debug draw", &_debug_draw_enabled))
                    {
                        ctx.api->set_debug_draw_enabled(_debug_draw_enabled);
                    }
                }

                bool runtime_rails_enabled = _orbit.runtime_orbiter_rails_enabled();
                if (ImGui::Checkbox("Runtime orbiter rails", &runtime_rails_enabled))
                {
                    _orbit.runtime_orbiter_rails_enabled() = runtime_rails_enabled;
                    mark_prediction_dirty();
                }
                double runtime_rails_distance_m = _orbit.runtime_orbiter_rails_distance_m();
                if (ImGui::DragScalar("Runtime rails distance (m)",
                                      ImGuiDataType_Double,
                                      &runtime_rails_distance_m,
                                      100.0f,
                                      nullptr,
                                      nullptr,
                                      "%.0f"))
                {
                    _orbit.runtime_orbiter_rails_distance_m() = std::max(0.0, runtime_rails_distance_m);
                }

                // ----------------------------------------------------------------
                // Contact log
                // ----------------------------------------------------------------
                ImGui::Separator();
                ImGui::Text("Contacts: %zu", _contact_log.size());

                const int max_lines = 6;
                const int n = static_cast<int>(std::min(_contact_log.size(), static_cast<size_t>(max_lines)));
                for (int i = 0; i < n; ++i)
                {
                    const ContactLogEntry &e = _contact_log[_contact_log.size() - 1 - static_cast<size_t>(i)];
                    ImGui::Text("[%s][%.2fs] self=%u other=%u depth=%.3f p=(%.2f,%.2f,%.2f)",
                                contact_event_type_name(e.type),
                                e.time_s,
                                e.self_body,
                                e.other_body,
                                e.penetration_depth,
                                e.point.x, e.point.y, e.point.z);
                }

                // ----------------------------------------------------------------
                // Ship controller HUD
                // ----------------------------------------------------------------
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
                {
                    const EntityId player_eid = _orbit.player_entity();
                    if (player_eid.is_valid())
                    {
                        Entity *player = _world.entities().find(player_eid);
                        if (player)
                        {
                            auto *sc = player->get_component<ShipController>();
                            if (sc)
                            {
                                ImGui::Separator();
                                const bool rails_warp =
                                        _orbital_physics.rails_warp_active() &&
                                        _time_warp.mode == TimeWarpState::Mode::RailsWarp;
                                const glm::vec3 td = rails_warp
                                                             ? _orbital_physics.rails_last_thrust_dir_local()
                                                             : sc->last_thrust_dir();
                                ImGui::Text("SAS: %s  [T] toggle", sc->sas_enabled() ? "ON " : "OFF");
                                ImGui::Text("Thrust input: (%.1f, %.1f, %.1f)%s",
                                            td.x, td.y, td.z,
                                            (rails_warp && _orbital_physics.rails_thrust_applied_this_tick()) ? " [applied]" : "");

                                if (rails_warp)
                                {
                                    WorldVec3 ship_pos_world{0.0, 0.0, 0.0};
                                    glm::dvec3 ship_vel_world(0.0);
                                    glm::vec3 ship_vel_local_f(0.0f);
                                    if (prediction_subjects.get_player_world_state(ship_pos_world,
                                                                                   ship_vel_world,
                                                                                   ship_vel_local_f))
                                    {
                                        ImGui::Text("Speed(world): %.2f m/s", glm::length(ship_vel_world));
                                    }
                                }
                                else if (player->has_physics() && _physics)
                                {
                                    const Physics::BodyId body_id{player->physics_body_value()};
                                    if (_physics->is_body_valid(body_id))
                                    {
                                        const Physics::MotionType motion = _physics->get_motion_type(body_id);
                                        const char *motion_str =
                                                (motion == Physics::MotionType::Dynamic)
                                                    ? "Dynamic"
                                                    : (motion == Physics::MotionType::Kinematic) ? "Kinematic (forces ignored)" : "Static";
                                        ImGui::Text("Motion: %s", motion_str);

                                        float thrust = sc->thrust_force();
                                        if (ImGui::DragFloat("Thrust force (N)", &thrust, 1000.0f, 0.0f, 1.0e9f, "%.1f"))
                                        {
                                            sc->set_thrust_force(thrust);
                                        }

                                        float torque = sc->torque_strength();
                                        if (ImGui::DragFloat("Torque strength (N*m)", &torque, 1000.0f, 0.0f, 1.0e9f, "%.1f"))
                                        {
                                            sc->set_torque_strength(torque);
                                        }

                                        float sas = sc->sas_damping();
                                        if (ImGui::DragFloat("SAS damping", &sas, 0.1f, 0.0f, 1.0e4f, "%.2f"))
                                        {
                                            sc->set_sas_damping(sas);
                                        }

                                        const glm::vec3 vel = _physics->get_linear_velocity(body_id);
                                        ImGui::Text("Speed(local): %.2f m/s", glm::length(vel));
                                        if (_physics_context)
                                        {
                                            const glm::dvec3 v_world = _physics_context->velocity_origin_world() + glm::dvec3(vel);
                                            ImGui::Text("Speed(world): %.2f m/s", glm::length(v_world));
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
#endif

                {
                    const EntityId player_eid = _orbit.player_entity();
                    if (player_eid.is_valid())
                    {
                        const Entity *player = _world.entities().find(player_eid);
                        if (player)
                        {
                            ImGui::Separator();
                            const glm::vec3 com_local = player->physics_origin_offset_local();
                            const float alpha_f = std::clamp(ctx.interpolation_alpha(), 0.0f, 1.0f);
                            const WorldVec3 model_origin_world = player->get_render_position_world(alpha_f);
                            const WorldVec3 com_world = player->get_render_physics_center_of_mass_world(alpha_f);
                            ImGui::Text("Model origin(world): %.2f, %.2f, %.2f",
                                        model_origin_world.x, model_origin_world.y, model_origin_world.z);
                            ImGui::Text("COM local: %.3f, %.3f, %.3f",
                                        com_local.x, com_local.y, com_local.z);
                            ImGui::Text("COM world: %.2f, %.2f, %.2f",
                                        com_world.x, com_world.y, com_world.z);
                        }
                    }
                }

                // ----------------------------------------------------------------
                // Subject summary and physics debug stay in the HUD; detailed orbit diagnostics live in N Body Orbit Debug.
                // ----------------------------------------------------------------
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
                ImGui::Separator();
                if (_physics && _physics_context && ImGui::CollapsingHeader("Physics Debug"))
                {
                    using VelocityOriginMode = OrbitalPhysicsSystem::VelocityOriginMode;
                    int mode_idx = (_orbital_physics.velocity_origin_mode() == VelocityOriginMode::PerStepAnchorSync) ? 0 : 1;
                    const char *modes[] = {"Per-step anchor sync", "Free-fall anchor frame"};
                    if (ImGui::Combo("Velocity origin mode", &mode_idx, modes, IM_ARRAYSIZE(modes)))
                    {
                        _orbital_physics.set_velocity_origin_mode(
                                (mode_idx == 0)
                                    ? VelocityOriginMode::PerStepAnchorSync
                                    : VelocityOriginMode::FreeFallAnchorFrame);
                        mark_prediction_dirty();
                    }

                    GameWorld::RebaseSettings rs = _world.rebase_settings();
                    float v_rebase = static_cast<float>(rs.velocity_threshold_mps);
                    if (ImGui::DragFloat("Velocity rebase threshold (m/s)", &v_rebase, 50.0f, 0.0f, 100000.0f, "%.1f"))
                    {
                        rs.velocity_threshold_mps = static_cast<double>(std::max(0.0f, v_rebase));
                        _world.set_rebase_settings(rs);
                    }
                    ImGui::SameLine();
                    ImGui::TextUnformatted("(0 = off)");

                    const EntityId player_eid = _orbit.player_entity();
                    if (_physics && player_eid.is_valid())
                    {
                        const Entity *player = _world.entities().find(player_eid);
                        if (player && player->has_physics())
                        {
                            const Physics::BodyId body_id{player->physics_body_value()};
                            if (_physics->is_body_valid(body_id))
                            {
                                const Physics::MotionType motion = _physics->get_motion_type(body_id);
                                bool kinematic = motion == Physics::MotionType::Kinematic;
                                if (ImGui::Checkbox("Controlled vessel kinematic", &kinematic))
                                {
                                    const Physics::MotionType target =
                                            kinematic ? Physics::MotionType::Kinematic : Physics::MotionType::Dynamic;
                                    (void) _physics->set_motion_type(body_id, target);
                                }
                                ImGui::SameLine();
                                ImGui::TextUnformatted("Controlled vessel is also the rebase anchor.");
                            }
                        }
                    }
                }
#endif
            }
            ImGui::End();
        }

        if (spacecraft_orbit_prediction_uses_nbody())
        {
            ManeuverUiController::Context maneuver_ui = build_maneuver_ui_context(ctx);
            ManeuverUiController::open_nodes_panel_from_orbit_pick_release(maneuver_ui);

            if (_show_maneuver_nodes_panel)
            {
                ManeuverUiController::draw_nodes_panel(maneuver_ui);
            }
            ManeuverUiController::draw_imgui_gizmo(maneuver_ui);
        }
        if (spacecraft_orbit_prediction_uses_kepler())
        {
            KeplerManeuverGizmoViewContext kepler_gizmo_view{};
            if (_kepler_prediction &&
                build_kepler_maneuver_gizmo_view_context(ctx, kepler_gizmo_view))
            {
                KeplerManeuverGizmoDrawContext kepler_gizmo{
                        .ctx = ctx,
                        .maneuver = _kepler_maneuver,
                        .view = kepler_gizmo_view,
                        .apply_command = [this](const KeplerManeuverCommand &command) {
                            return apply_kepler_maneuver_command(command);
                        },
                };
                KeplerManeuverGizmo::draw(kepler_gizmo);
            }
            update_kepler_maneuver_orbit_pick_creation(ctx);
            if (_kepler_prediction && _kepler_prediction->state().dirty)
            {
                update_kepler_prediction(ctx);
                draw_kepler_prediction(ctx);
            }
        }
        if (_show_nbody_orbit_debug && spacecraft_orbit_prediction_uses_nbody())
        {
            draw_nbody_orbit_debug_window(ctx);
        }
        if (_show_kepler_orbit_debug && spacecraft_orbit_prediction_uses_kepler())
        {
            draw_kepler_orbit_debug_window(ctx);
        }
        if (_show_frame_view)
        {
            _frame_monitor.draw_ui();
        }
    }

} // namespace Game
