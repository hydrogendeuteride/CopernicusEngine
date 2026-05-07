#include "game/states/gameplay/prediction_kepler/kepler_prediction_draw.h"

#include "core/picking/picking_system.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_system.h"

#include <cmath>

namespace Game
{
    namespace
    {
        bool finite_world(const WorldVec3 &p)
        {
            return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
        }

        void emit_line_set(OrbitPlotSystem &orbit_plot,
                           PickingSystem *picking,
                           const bool emit_pick,
                           const char *pick_owner_name,
                           const KeplerOrbitLineSet &line_set,
                           const glm::vec4 &color,
                           const OrbitPlotDepth depth)
        {
            if (!line_set.valid || line_set.vertices.size() < 2u)
            {
                return;
            }

            uint32_t pick_group = 0u;
            const bool pick_enabled = emit_pick && picking != nullptr && pick_owner_name != nullptr;
            if (pick_enabled)
            {
                pick_group = picking->add_line_pick_group(pick_owner_name);
            }

            for (std::size_t i = 1; i < line_set.vertices.size(); ++i)
            {
                const KeplerOrbitLineVertex &a = line_set.vertices[i - 1u];
                const KeplerOrbitLineVertex &b = line_set.vertices[i];
                if (!finite_world(a.position_world) || !finite_world(b.position_world))
                {
                    continue;
                }

                orbit_plot.add_line(a.position_world, b.position_world, color, depth);
                if (pick_enabled)
                {
                    picking->add_line_pick_segment(
                            pick_group,
                            a.position_world,
                            b.position_world,
                            a.t_s,
                            b.t_s);
                }
            }
        }

        glm::vec4 track_base_color(const KeplerPredictionState::Track &track,
                                   const KeplerPredictionDrawContext &context)
        {
            const float alpha =
                    track.celestial_nbody ? 0.42f : (track.celestial ? 0.58f : context.base_color.a);
            return glm::vec4(track.orbit_rgb, alpha);
        }
    } // namespace

    void draw_kepler_prediction(const KeplerPredictionState &state,
                                const KeplerPredictionDrawContext &context)
    {
        if (!state.enabled || !state.valid || !context.orbit_plot)
        {
            return;
        }

        if (!state.tracks.empty())
        {
            for (const KeplerPredictionState::Track &track : state.tracks)
            {
                if (!track.valid)
                {
                    continue;
                }
                if (track.celestial_nbody && !context.draw_celestial_nbody_tracks)
                {
                    continue;
                }
                if (track.celestial && !track.celestial_nbody && !context.draw_celestial_kepler_tracks)
                {
                    continue;
                }
                if (!track.celestial && !context.draw_orbiter_tracks)
                {
                    continue;
                }

                emit_line_set(*context.orbit_plot,
                              context.picking,
                              context.emit_pick && !track.celestial,
                              track.label.empty() ? "KeplerOrbit/Base" : track.label.c_str(),
                              track.base_lines,
                              track_base_color(track, context),
                              context.depth);

                if (context.draw_planned && track.active_player)
                {
                    emit_line_set(*context.orbit_plot,
                                  context.picking,
                                  context.emit_pick,
                                  track.label.empty() ? "KeplerOrbit/Planned" : track.label.c_str(),
                                  track.planned_lines,
                                  context.planned_color,
                                  context.depth);
                }
            }
            return;
        }

        emit_line_set(*context.orbit_plot,
                      context.picking,
                      context.emit_pick,
                      "KeplerOrbit/Base",
                      state.base_lines,
                      context.base_color,
                      context.depth);

        if (context.draw_planned)
        {
            emit_line_set(*context.orbit_plot,
                          context.picking,
                          context.emit_pick,
                          "KeplerOrbit/Planned",
                          state.planned_lines,
                          context.planned_color,
                          context.depth);
        }
    }

    void KeplerPredictionSystem::draw(const KeplerPredictionDrawContext &context) const
    {
        draw_kepler_prediction(_state, context);
    }
} // namespace Game
