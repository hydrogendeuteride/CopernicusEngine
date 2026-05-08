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

        KeplerPredictionUpdateContext kepler_context{};
        kepler_context.orbit = &_orbit;
        kepler_context.world = &_world;
        kepler_context.physics = _physics.get();
        kepler_context.physics_context = _physics_context.get();
        kepler_context.scenario_config = &_scenario_config;
        kepler_context.enabled = _prediction->state().enabled;
        kepler_context.current_sim_time_s = current_sim_time_s();
        kepler_context.options = _kepler_prediction_options;
        kepler_context.tessellation = _kepler_tessellation_options;
        kepler_context.build_celestial_kepler_tracks = _kepler_draw_celestial_kepler_tracks;
        kepler_context.build_celestial_nbody_tracks = _kepler_draw_celestial_nbody_tracks;

        const int render_segment_budget = _prediction->budget().render_max_segments_cpu;
        if (render_segment_budget > 1)
        {
            constexpr std::size_t kKeplerCurveSourceVertexCap = 16384u;
            kepler_context.tessellation.max_vertices_total =
                    std::min(kepler_context.tessellation.max_vertices_total,
                             static_cast<std::size_t>(std::min(render_segment_budget + 1,
                                                               static_cast<int>(kKeplerCurveSourceVertexCap))));
            kepler_context.tessellation.max_vertices_per_arc =
                    std::min(kepler_context.tessellation.max_vertices_per_arc,
                             kepler_context.tessellation.max_vertices_total);
        }

        _kepler_prediction->update(kepler_context);
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
        _kepler_prediction->draw(kepler_draw);
    }
} // namespace Game
