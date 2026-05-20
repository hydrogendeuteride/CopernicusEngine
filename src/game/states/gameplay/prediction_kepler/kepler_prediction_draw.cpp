#include "game/states/gameplay/prediction_kepler/kepler_prediction_draw.h"

#include "core/picking/picking_system.h"
#include "game/orbit/kepler/kepler_draw_lod_line_builder.h"
#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_orbit_pick.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_system.h"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <span>
#include <string>
#include <vector>

namespace Game
{
    namespace
    {
        // Local draw-path tuning knobs, grouped by the subsystem that consumes them.
        struct PickSegmentConfig
        {
            static constexpr std::size_t max_segments_per_line_set = 4'096u;
        };

        struct DashShaderConfig
        {
            static constexpr double on_px = 14.0;
            static constexpr double off_px = 9.0;
            static constexpr double period_px = on_px + off_px;
        };

        struct ClipConfig
        {
            static constexpr double plane_epsilon = 1.0e-4;
            static constexpr double direction_epsilon = 1.0e-12;
        };

        bool finite_world(const WorldVec3 &p)
        {
            return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
        }

        std::string kepler_orbit_pick_owner(const KeplerManeuverOrbitPickRole role,
                                            const std::string &track_label)
        {
            std::string owner = (role == KeplerManeuverOrbitPickRole::Planned)
                                        ? "KeplerOrbit/Planned"
                                        : "KeplerOrbit/Base";
            if (!track_label.empty())
            {
                owner += "/";
                owner += track_label;
            }
            return owner;
        }

        std::size_t emit_pick_segments(PickingSystem &picking,
                                       const uint32_t pick_group,
                                       const KeplerArcLineSet &line_set,
                                       std::vector<Picking::LinePickSegmentData> *scratch)
        {
            // Picking is capped independently of render vertices so dense orbit
            // lines stay drawable without flooding the picking system.
            const std::size_t requested_segments =
                    (line_set.vertices.size() > 1u) ? line_set.vertices.size() - 1u : 0u;
            const std::size_t target_segments =
                    std::min(requested_segments, PickSegmentConfig::max_segments_per_line_set);
            if (target_segments == 0u)
            {
                return 0u;
            }

            std::size_t emitted_segments = 0u;
            const auto append_segment = [&](const std::size_t a_index,
                                            const std::size_t b_index) {
                if (a_index >= line_set.vertices.size() ||
                    b_index >= line_set.vertices.size() ||
                    b_index <= a_index)
                {
                    return;
                }

                const KeplerArcLineVertex &a = line_set.vertices[a_index];
                const KeplerArcLineVertex &b = line_set.vertices[b_index];
                if (!finite_world(a.position_world) || !finite_world(b.position_world))
                {
                    return;
                }

                if (scratch)
                {
                    scratch->push_back(Picking::LinePickSegmentData{
                            .a_world = a.position_world,
                            .b_world = b.position_world,
                            .a_time_s = a.t_s,
                            .b_time_s = b.t_s,
                    });
                }
                else
                {
                    picking.add_line_pick_segment(pick_group,
                                                  a.position_world,
                                                  b.position_world,
                                                  a.t_s,
                                                  b.t_s);
                }
                ++emitted_segments;
            };

            const std::size_t batch_begin = scratch ? scratch->size() : 0u;
            if (requested_segments <= target_segments)
            {
                for (std::size_t i = 1u; i < line_set.vertices.size(); ++i)
                {
                    append_segment(i - 1u, i);
                }
            }
            else
            {
                const double source_span = static_cast<double>(requested_segments);
                for (std::size_t i = 0u; i < target_segments; ++i)
                {
                    const double a_u = static_cast<double>(i) / static_cast<double>(target_segments);
                    const double b_u = static_cast<double>(i + 1u) / static_cast<double>(target_segments);
                    std::size_t a_index =
                            static_cast<std::size_t>(std::llround(a_u * source_span));
                    std::size_t b_index =
                            static_cast<std::size_t>(std::llround(b_u * source_span));
                    a_index = std::min(a_index, line_set.vertices.size() - 1u);
                    b_index = std::min(b_index, line_set.vertices.size() - 1u);
                    if (b_index <= a_index && a_index + 1u < line_set.vertices.size())
                    {
                        b_index = a_index + 1u;
                    }
                    append_segment(a_index, b_index);
                }
            }

            if (scratch && emitted_segments > 0u)
            {
                picking.add_line_pick_segments(
                        pick_group,
                        std::span<const PickingSystem::LinePickSegmentData>(scratch->data() + batch_begin,
                                                                            emitted_segments));
            }
            return emitted_segments;
        }

        double dash_period_scale(const KeplerPredictionDrawContext &context)
        {
            double scale = 1.0;
            const double source_period_px = context.dashed_segment_on_px + context.dashed_segment_off_px;
            if (std::isfinite(source_period_px) && source_period_px > 1.0e-6)
            {
                scale = DashShaderConfig::period_px / source_period_px;
            }
            return scale;
        }

        double clip_plane_distance(const glm::dvec4 &clip, const int plane_index)
        {
            switch (plane_index)
            {
                case 0: return clip.x + clip.w;
                case 1: return -clip.x + clip.w;
                case 2: return clip.y + clip.w;
                case 3: return -clip.y + clip.w;
                case 4: return clip.w - clip.z;
                default: return 0.0;
            }
        }

        bool clip_segment_to_view_t(const glm::dvec4 &clip_a,
                                    const glm::dvec4 &clip_b,
                                    double &out_t0,
                                    double &out_t1)
        {
            if (!std::isfinite(clip_a.x) || !std::isfinite(clip_a.y) ||
                !std::isfinite(clip_a.z) || !std::isfinite(clip_a.w) ||
                !std::isfinite(clip_b.x) || !std::isfinite(clip_b.y) ||
                !std::isfinite(clip_b.z) || !std::isfinite(clip_b.w))
            {
                return false;
            }

            double t0 = 0.0;
            double t1 = 1.0;
            for (int plane_index = 0; plane_index < 5; ++plane_index)
            {
                const double d0 = clip_plane_distance(clip_a, plane_index);
                const double d1 = clip_plane_distance(clip_b, plane_index);
                const bool a_inside = d0 >= ClipConfig::plane_epsilon;
                const bool b_inside = d1 >= ClipConfig::plane_epsilon;
                if (a_inside && b_inside)
                {
                    continue;
                }
                if (!a_inside && !b_inside)
                {
                    return false;
                }

                const double denom = d1 - d0;
                if (!(std::abs(denom) > ClipConfig::direction_epsilon) || !std::isfinite(denom))
                {
                    return false;
                }

                const double t = std::clamp((ClipConfig::plane_epsilon - d0) / denom, 0.0, 1.0);
                if (!a_inside)
                {
                    t0 = std::max(t0, t);
                }
                else
                {
                    t1 = std::min(t1, t);
                }
                if (t0 > t1)
                {
                    return false;
                }
            }

            out_t0 = t0;
            out_t1 = t1;
            return true;
        }

        glm::dvec4 project_world_to_clip(const KeplerPredictionDrawContext &context,
                                         const WorldVec3 &world)
        {
            const glm::dvec3 local = world_to_local_d(world, context.world_origin);
            return glm::dmat4(context.viewproj) * glm::dvec4(local, 1.0);
        }

        struct ClippedWorldSegment
        {
            WorldVec3 a_world{0.0, 0.0, 0.0};
            WorldVec3 b_world{0.0, 0.0, 0.0};
            double projected_length_px{0.0};
        };

        double projected_clip_segment_px(const KeplerPredictionDrawContext &context,
                                         const glm::dvec4 &clip_a,
                                         const glm::dvec4 &clip_b)
        {
            if (!(context.viewport_width_px > 0.0) ||
                !(context.viewport_height_px > 0.0) ||
                !std::isfinite(context.viewport_width_px) ||
                !std::isfinite(context.viewport_height_px) ||
                !(std::abs(clip_a.w) > 1.0e-9) ||
                !(std::abs(clip_b.w) > 1.0e-9) ||
                !std::isfinite(clip_a.w) ||
                !std::isfinite(clip_b.w))
            {
                return 0.0;
            }

            const glm::dvec2 ndc_a = glm::dvec2(clip_a) / clip_a.w;
            const glm::dvec2 ndc_b = glm::dvec2(clip_b) / clip_b.w;
            if (!std::isfinite(ndc_a.x) ||
                !std::isfinite(ndc_a.y) ||
                !std::isfinite(ndc_b.x) ||
                !std::isfinite(ndc_b.y))
            {
                return 0.0;
            }

            const glm::dvec2 pixel_delta =
                    (ndc_b - ndc_a) *
                    glm::dvec2(0.5 * context.viewport_width_px,
                               0.5 * context.viewport_height_px);
            const double length_px = glm::length(pixel_delta);
            return (std::isfinite(length_px) && length_px > 0.0)
                           ? length_px * dash_period_scale(context)
                           : 0.0;
        }

        bool clip_world_segment_to_view(const KeplerPredictionDrawContext &context,
                                        const WorldVec3 &a_world,
                                        const WorldVec3 &b_world,
                                        const bool measure_projected_length,
                                        ClippedWorldSegment &out)
        {
            // Orbit vertices can span huge distances. Clip before submitting so
            // downstream line rendering and dash measurement operate on visible spans.
            const glm::dvec4 clip_a = project_world_to_clip(context, a_world);
            const glm::dvec4 clip_b = project_world_to_clip(context, b_world);
            double t0 = 0.0;
            double t1 = 1.0;
            if (!clip_segment_to_view_t(clip_a, clip_b, t0, t1))
            {
                return false;
            }

            const glm::dvec4 clipped_a = glm::mix(clip_a, clip_b, t0);
            const glm::dvec4 clipped_b = glm::mix(clip_a, clip_b, t1);
            out.a_world = glm::mix(a_world, b_world, t0);
            out.b_world = glm::mix(a_world, b_world, t1);
            if (!finite_world(out.a_world) || !finite_world(out.b_world))
            {
                return false;
            }

            out.projected_length_px = measure_projected_length
                                              ? projected_clip_segment_px(context, clipped_a, clipped_b)
                                              : 0.0;
            return true;
        }

        void advance_dash_cursor(double &cursor_px, const double segment_px)
        {
            if (!std::isfinite(segment_px) || !(segment_px > 0.0))
            {
                return;
            }

            cursor_px = std::fmod(cursor_px + segment_px, DashShaderConfig::period_px);
            if (cursor_px < 0.0)
            {
                cursor_px += DashShaderConfig::period_px;
            }
        }

        void dash_coords_for_segment(double &dash_cursor_px,
                                     const double segment_px,
                                     float &out_a_px,
                                     float &out_b_px)
        {
            const double start_px = std::fmod(dash_cursor_px, DashShaderConfig::period_px);
            const double safe_start_px = start_px < 0.0 ? start_px + DashShaderConfig::period_px : start_px;
            out_a_px = static_cast<float>(safe_start_px);
            out_b_px = static_cast<float>(safe_start_px + std::max(0.0, segment_px));
            advance_dash_cursor(dash_cursor_px, segment_px);
        }

        // Converts one built Kepler line set into renderer line commands, optional
        // overlay commands, and optional picking segments for maneuver interactions.
        void emit_line_set(OrbitPlotSystem &orbit_plot,
                           PickingSystem *picking,
                           const bool emit_pick,
                           const char *pick_owner_name,
                           const Picking::LinePickPayload pick_payload,
                           const KeplerArcLineSet &line_set,
                           const glm::vec4 &color,
                           const OrbitPlotDepth depth,
                           const OrbitPlotLineStyle style,
                           const KeplerPredictionDrawContext &draw_context,
                           const float line_overlay_boost,
                           std::vector<Picking::LinePickSegmentData> *pick_segment_scratch)
        {
            if (!line_set.valid || line_set.vertices.size() < 2u)
            {
                return;
            }

            uint32_t pick_group = 0u;
            const bool pick_enabled = emit_pick && picking != nullptr && pick_owner_name != nullptr;
            if (pick_enabled)
            {
                pick_group = picking->add_line_pick_group(pick_owner_name, pick_payload);
            }

            const std::size_t segment_count = line_set.vertices.size() - 1u;
            const bool overlay_enabled = line_overlay_boost > 0.0f && color.a > 0.0f;
            orbit_plot.reserve_pending_lines(segment_count * (overlay_enabled ? 2u : 1u));
            double dash_cursor_px = 0.0;
            for (std::size_t i = 1; i < line_set.vertices.size(); ++i)
            {
                const KeplerArcLineVertex &a = line_set.vertices[i - 1u];
                const KeplerArcLineVertex &b = line_set.vertices[i];
                if (!finite_world(a.position_world) || !finite_world(b.position_world))
                {
                    continue;
                }

                const bool measure_projected_length = style == OrbitPlotLineStyle::Dashed;
                ClippedWorldSegment draw_segment{};
                if (!clip_world_segment_to_view(draw_context,
                                                a.position_world,
                                                b.position_world,
                                                measure_projected_length,
                                                draw_segment))
                {
                    continue;
                }

                float dash_coord_a_px = -1.0f;
                float dash_coord_b_px = -1.0f;
                if (style == OrbitPlotLineStyle::Dashed)
                {
                    const double segment_px = draw_segment.projected_length_px;
                    dash_coords_for_segment(dash_cursor_px,
                                            std::isfinite(segment_px) ? segment_px : 0.0,
                                            dash_coord_a_px,
                                            dash_coord_b_px);
                }

                orbit_plot.add_line_command(OrbitPlotSystem::LineCommand{
                        .a_world = draw_segment.a_world,
                        .b_world = draw_segment.b_world,
                        .color = color,
                        .depth = depth,
                        .style = style,
                        .dash_coord_a_px = dash_coord_a_px,
                        .dash_coord_b_px = dash_coord_b_px,
                });
                if (line_overlay_boost > 0.0f)
                {
                    glm::vec4 overlay_color = color;
                    overlay_color.a = std::clamp(overlay_color.a * line_overlay_boost, 0.0f, 1.0f);
                    if (overlay_color.a > 0.0f)
                    {
                        orbit_plot.add_line_command(OrbitPlotSystem::LineCommand{
                                .a_world = draw_segment.a_world,
                                .b_world = draw_segment.b_world,
                                .color = overlay_color,
                                .depth = OrbitPlotDepth::AlwaysOnTop,
                                .style = style,
                                .dash_coord_a_px = dash_coord_a_px,
                                .dash_coord_b_px = dash_coord_b_px,
                        });
                    }
                }
            }
            if (pick_enabled)
            {
                emit_pick_segments(*picking, pick_group, line_set, pick_segment_scratch);
            }
        }

        glm::vec4 track_base_color(const KeplerPredictionState::Track &track,
                                   const KeplerPredictionDrawContext &context)
        {
            // Celestial tracks are reference context, so keep them less dominant
            // than spacecraft prediction lines.
            const float alpha =
                    track.celestial_nbody ? 0.42f : (track.celestial ? 0.58f : context.base_color.a);
            return glm::vec4(track.orbit_rgb, alpha);
        }

        KeplerArcLineOptions draw_line_options_for_track(
                const KeplerPredictionState::Track &track,
                const KeplerPredictionDrawContext &context)
        {
            KeplerArcLineOptions options =
                    track.celestial ? track.line_options : context.line_options;
            options.propagation = track.line_propagation;
            options.max_vertices_total = std::max<std::size_t>(options.max_vertices_total, 2u);
            options.max_vertices_per_arc = std::max<std::size_t>(options.max_vertices_per_arc, 2u);
            return options;
        }

        KeplerArcLineSet build_draw_lod_lines(
                const KeplerPredictionState::Track &track,
                const KeplerPredictionDrawContext &context,
                const std::span<const KeplerOrbitArc> arcs)
        {
            if (arcs.empty())
            {
                return {};
            }

            return build_kepler_draw_lod_lines(KeplerDrawLodLineBuildRequest{
                    .arcs = arcs,
                    .ephemeris = nullptr,
                    .body_state_provider = track.body_state_provider,
                    .world_frame = track.world_frame,
                    .line_options = draw_line_options_for_track(track, context),
                    .viewproj = context.viewproj,
                    .world_origin = context.world_origin,
                    .viewport_width_px = context.viewport_width_px,
                    .viewport_height_px = context.viewport_height_px,
                    .render_error_px = context.render_error_px,
            });
        }

        std::span<const KeplerOrbitArc> planned_draw_arcs(
                const KeplerPredictionState::Track &track)
        {
            if (track.planned_arcs.empty())
            {
                return {};
            }
            const std::size_t first =
                    std::min(track.first_planned_draw_arc_index, track.planned_arcs.size());
            return std::span<const KeplerOrbitArc>(
                    track.planned_arcs.data() + first,
                    track.planned_arcs.size() - first);
        }

        void reserve_pick_segment_scratch(const KeplerPredictionState &state,
                                          const KeplerPredictionDrawContext &context)
        {
            if (!context.pick_segment_scratch)
            {
                return;
            }

            context.pick_segment_scratch->clear();
            if (!context.emit_pick || !state.enabled || !state.valid)
            {
                return;
            }

            std::size_t target_segments = 0u;
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

                    if (!track.celestial && track.active_player)
                    {
                        target_segments += std::min(context.line_options.max_vertices_total,
                                                    PickSegmentConfig::max_segments_per_line_set);
                    }
                    if (context.draw_planned && track.active_player)
                    {
                        target_segments += std::min(context.line_options.max_vertices_total,
                                                    PickSegmentConfig::max_segments_per_line_set);
                    }
                }
            }

            context.pick_segment_scratch->reserve(target_segments);
        }
    } // namespace

    void draw_kepler_prediction(const KeplerPredictionState &state,
                                const KeplerPredictionDrawContext &context)
    {
        // The Kepler draw path owns orbit-plot and line-pick output for this frame.
        if (context.picking)
        {
            context.picking->clear_line_picks();
            context.picking->settings().enable_line_hover = context.emit_pick && state.enabled;
        }
        if (context.orbit_plot)
        {
            context.orbit_plot->clear_pending();
        }
        reserve_pick_segment_scratch(state, context);

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

                // Only active spacecraft tracks are pickable; celestial tracks are
                // visual context and should not create maneuver handles.
                const bool base_pick_enabled =
                        context.emit_pick && track.active_player && !track.celestial;
                std::string base_owner{};
                Picking::LinePickPayload base_payload{};
                if (base_pick_enabled)
                {
                    base_owner = kepler_orbit_pick_owner(KeplerManeuverOrbitPickRole::Base,
                                                         track.label);
                    base_payload = KeplerManeuverPick::make_payload(KeplerManeuverOrbitPickRole::Base,
                                                                    track.entity);
                }
                KeplerArcLineSet draw_base_lines{};
                const KeplerArcLineSet *base_lines = &track.prebuilt_base_lines;
                if (!track.celestial_nbody)
                {
                    draw_base_lines = build_draw_lod_lines(
                            track,
                            context,
                            std::span<const KeplerOrbitArc>(track.base_arcs.data(),
                                                            track.base_arcs.size()));
                    base_lines = &draw_base_lines;
                }
                emit_line_set(*context.orbit_plot,
                              context.picking,
                              base_pick_enabled,
                              base_pick_enabled ? base_owner.c_str() : nullptr,
                              base_payload,
                              *base_lines,
                              track_base_color(track, context),
                              context.depth,
                              OrbitPlotLineStyle::Solid,
                              context,
                              context.line_overlay_boost,
                              context.pick_segment_scratch);

                if (context.draw_planned && track.active_player)
                {
                    const std::span<const KeplerOrbitArc> planned_arcs = planned_draw_arcs(track);
                    if (planned_arcs.empty())
                    {
                        continue;
                    }
                    const KeplerArcLineSet planned_lines =
                            build_draw_lod_lines(track, context, planned_arcs);
                    const bool planned_pick_enabled = context.emit_pick;
                    std::string planned_owner{};
                    Picking::LinePickPayload planned_payload{};
                    if (planned_pick_enabled)
                    {
                        planned_owner = kepler_orbit_pick_owner(KeplerManeuverOrbitPickRole::Planned,
                                                                track.label);
                        planned_payload = KeplerManeuverPick::make_payload(
                                KeplerManeuverOrbitPickRole::Planned,
                                track.entity);
                    }
                    const float planned_overlay_boost =
                            std::max(context.line_overlay_boost, context.planned_line_overlay_boost);
                    emit_line_set(*context.orbit_plot,
                                  context.picking,
                                  planned_pick_enabled,
                                  planned_pick_enabled ? planned_owner.c_str() : nullptr,
                                  planned_payload,
                                  planned_lines,
                                  context.planned_color,
                                  context.depth,
                                  context.draw_planned_as_dashed ? OrbitPlotLineStyle::Dashed
                                                                 : OrbitPlotLineStyle::Solid,
                                  context,
                                  planned_overlay_boost,
                                  context.pick_segment_scratch);
                }
            }
            return;
        }
    }

    void KeplerPredictionSystem::draw(const KeplerPredictionDrawContext &context) const
    {
        KeplerPredictionDrawContext draw_context = context;
        draw_context.pick_segment_scratch = &_pick_segment_scratch;
        draw_kepler_prediction(_state, draw_context);
    }
} // namespace Game
