#include "game/states/gameplay/gameplay_state.h"

#include "core/engine.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_system.h"
#include "game/states/gameplay/prediction_nbody/prediction_system.h"

#include <algorithm>
#include <cstddef>

namespace Game
{
    void GameplayState::mark_kepler_prediction_dirty()
    {
        if (_kepler_prediction)
        {
            _kepler_prediction->mark_dirty();
        }
    }

    void GameplayState::clear_kepler_prediction_runtime()
    {
        if (_kepler_prediction)
        {
            _kepler_prediction->reset();
        }
        _kepler_maneuver.clear_node_display_states();
    }

    void GameplayState::update_kepler_prediction(GameStateContext &ctx)
    {
        (void) ctx;
        if (!_kepler_prediction)
        {
            return;
        }
        if (!spacecraft_orbit_prediction_uses_kepler())
        {
            clear_kepler_prediction_runtime();
            return;
        }

        const double sim_time_s = current_sim_time_s();
        (void) apply_kepler_maneuver_command(
                KeplerManeuverCommand::prune_past_nodes(sim_time_s));

        KeplerPredictionUpdateContext kepler_context{};
        kepler_context.orbit = &_orbit;
        kepler_context.world = &_world;
        kepler_context.physics = _physics.get();
        kepler_context.physics_context = _physics_context.get();
        kepler_context.scenario_config = &_scenario_config;
        kepler_context.enabled = _prediction->state().enabled;
        kepler_context.current_sim_time_s = sim_time_s;
        kepler_context.current_wall_time_s = static_cast<double>(_elapsed);
        kepler_context.options = _kepler_prediction_options;
        kepler_context.line_options = _kepler_arc_line_options;
        kepler_context.maneuver_nodes = _kepler_maneuver.prediction_nodes();
        kepler_context.maneuver_revision = _kepler_maneuver.revision();
        kepler_context.build_celestial_kepler_tracks = _kepler_draw_celestial_kepler_tracks;
        kepler_context.build_celestial_nbody_tracks = _kepler_draw_celestial_nbody_tracks;

        const int render_segment_budget = _prediction->budget().render_max_segments_cpu;
        if (render_segment_budget > 1)
        {
            constexpr std::size_t kKeplerCurveSourceVertexCap = 16384u;
            kepler_context.line_options.max_vertices_total =
                    std::min(kepler_context.line_options.max_vertices_total,
                             static_cast<std::size_t>(std::min(render_segment_budget + 1,
                                                               static_cast<int>(kKeplerCurveSourceVertexCap))));
            kepler_context.line_options.max_vertices_per_arc =
                    std::min(kepler_context.line_options.max_vertices_per_arc,
                             kepler_context.line_options.max_vertices_total);
        }

        _kepler_prediction->update(kepler_context);
        _kepler_maneuver.resolve_node_display_states(_kepler_prediction->state());
    }

    void GameplayState::draw_kepler_prediction(GameStateContext &ctx)
    {
        if (!_kepler_prediction || !spacecraft_orbit_prediction_uses_kepler())
        {
            return;
        }

        KeplerPredictionDrawContext kepler_draw{};
        kepler_draw.orbit_plot =
                (ctx.renderer && ctx.renderer->_context) ? ctx.renderer->_context->orbit_plot : nullptr;
        kepler_draw.picking = (ctx.renderer != nullptr) ? ctx.renderer->picking() : nullptr;
        kepler_draw.draw_orbiter_tracks = _kepler_draw_orbiter_tracks;
        kepler_draw.draw_celestial_kepler_tracks = _kepler_draw_celestial_kepler_tracks;
        kepler_draw.draw_celestial_nbody_tracks = _kepler_draw_celestial_nbody_tracks;
        if (_prediction)
        {
            kepler_draw.planned_color = _prediction->state().draw_config.palette.orbit_planned;
            kepler_draw.draw_planned_as_dashed = _prediction->state().draw_config.draw_planned_as_dashed;
            kepler_draw.dashed_segment_on_px = _prediction->state().draw_config.dashed_segment_on_px;
            kepler_draw.dashed_segment_off_px = _prediction->state().draw_config.dashed_segment_off_px;
        }
        if (ctx.renderer && ctx.renderer->_sceneManager)
        {
            kepler_draw.viewproj = ctx.renderer->_sceneManager->getSceneData().viewproj;
            kepler_draw.world_origin = ctx.renderer->_sceneManager->get_world_origin();
            if (ctx.renderer->_logicalRenderExtent.width > 0)
            {
                kepler_draw.viewport_width_px = static_cast<double>(ctx.renderer->_logicalRenderExtent.width);
            }
            if (ctx.renderer->_logicalRenderExtent.height > 0)
            {
                kepler_draw.viewport_height_px = static_cast<double>(ctx.renderer->_logicalRenderExtent.height);
            }
        }
        kepler_draw.depth = OrbitPlotDepth::DepthTested;
        kepler_draw.line_overlay_boost = 0.0f;
        kepler_draw.planned_line_overlay_boost = 0.0f;
        _kepler_prediction->draw(kepler_draw);
    }

    KeplerManeuverCommandResult GameplayState::apply_kepler_maneuver_command(
            const KeplerManeuverCommand &command)
    {
        KeplerManeuverCommandResult result = _kepler_maneuver.apply_command(command);
        if (result.prediction_dirty)
        {
            mark_kepler_prediction_dirty();
        }
        return result;
    }
} // namespace Game
