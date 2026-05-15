#include "game/states/gameplay/prediction_kepler/kepler_prediction_draw.h"

#include "core/picking/picking_system.h"
#include "core/util/logger.h"
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
        struct PickLogConfig
        {
            static constexpr uint64_t summary_interval = 300u;
            static constexpr uint64_t large_interval = 60u;
            static constexpr std::size_t large_segment_count = 8'192u;
            static constexpr std::size_t warn_segment_count = 32'768u;
        };

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

        // Per-frame diagnostics for the line-picking path. These counters are
        // intentionally local to drawing so invalid visualization data is visible
        // without coupling it back into the prediction builder.
        struct KeplerPickFrameStats
        {
            uint64_t draw_call_id{0u};
            std::size_t groups{0u};
            std::size_t vertices{0u};
            std::size_t requested_segments{0u};
            std::size_t registered_segments{0u};
            std::size_t skipped_nonfinite_position_segments{0u};
            std::size_t invalid_time_segments{0u};
            std::size_t nonfinite_length_segments{0u};
            std::size_t largest_line_set_segments{0u};
            uint32_t max_group_id{0u};
            std::string largest_line_set_owner{};
            std::string largest_line_set_role{};
            double min_t_s{0.0};
            double max_t_s{0.0};
            double max_segment_length_m{0.0};
            bool have_finite_time{false};

            void record_time(const double t_s)
            {
                if (!std::isfinite(t_s))
                {
                    return;
                }
                if (!have_finite_time)
                {
                    min_t_s = t_s;
                    max_t_s = t_s;
                    have_finite_time = true;
                    return;
                }
                min_t_s = std::min(min_t_s, t_s);
                max_t_s = std::max(max_t_s, t_s);
            }
        };

        void record_pick_frame_line_set(KeplerPickFrameStats &stats,
                                        const uint32_t pick_group,
                                        const char *pick_owner_name,
                                        const char *line_role,
                                        const std::size_t vertices,
                                        const std::size_t requested_segments,
                                        const std::size_t registered_segments,
                                        const std::size_t skipped_nonfinite_position_segments,
                                        const std::size_t invalid_time_segments,
                                        const std::size_t nonfinite_length_segments,
                                        const bool have_finite_time,
                                        const double min_t_s,
                                        const double max_t_s,
                                        const double max_segment_length_m)
        {
            ++stats.groups;
            stats.max_group_id = std::max(stats.max_group_id, pick_group);
            stats.vertices += vertices;
            stats.requested_segments += requested_segments;
            stats.registered_segments += registered_segments;
            stats.skipped_nonfinite_position_segments += skipped_nonfinite_position_segments;
            stats.invalid_time_segments += invalid_time_segments;
            stats.nonfinite_length_segments += nonfinite_length_segments;
            if (have_finite_time)
            {
                stats.record_time(min_t_s);
                stats.record_time(max_t_s);
            }
            stats.max_segment_length_m = std::max(stats.max_segment_length_m, max_segment_length_m);
            if (registered_segments > stats.largest_line_set_segments)
            {
                stats.largest_line_set_segments = registered_segments;
                stats.largest_line_set_owner = pick_owner_name ? pick_owner_name : "(null)";
                stats.largest_line_set_role = line_role ? line_role : "(null)";
            }
        }

        void log_pick_frame_stats(const KeplerPickFrameStats &stats)
        {
            if (stats.groups == 0u)
            {
                return;
            }

            const bool group_ids_accumulated = static_cast<std::size_t>(stats.max_group_id) + 1u > stats.groups;
            const bool suspicious =
                    stats.registered_segments >= PickLogConfig::warn_segment_count ||
                    stats.skipped_nonfinite_position_segments > 0u ||
                    stats.invalid_time_segments > 0u ||
                    stats.nonfinite_length_segments > 0u ||
                    group_ids_accumulated;
            const bool periodic_log = (stats.draw_call_id % PickLogConfig::summary_interval) == 1u;
            const bool large_log =
                    stats.registered_segments >= PickLogConfig::large_segment_count &&
                    (stats.draw_call_id % PickLogConfig::large_interval) == 1u;
            if (!suspicious && !periodic_log && !large_log)
            {
                return;
            }

            if (suspicious)
            {
                Logger::warn("[KeplerPick] draw={} groups={} group_max={} segments={} requested={} vertices={} "
                             "largest='{}'/{}/{} skipped_pos={} invalid_time={} nonfinite_length={} "
                             "t_valid={} t=[{:.3f},{:.3f}] max_segment_m={:.3f}",
                             stats.draw_call_id,
                             stats.groups,
                             stats.max_group_id,
                             stats.registered_segments,
                             stats.requested_segments,
                             stats.vertices,
                             stats.largest_line_set_owner,
                             stats.largest_line_set_role,
                             stats.largest_line_set_segments,
                             stats.skipped_nonfinite_position_segments,
                             stats.invalid_time_segments,
                             stats.nonfinite_length_segments,
                             stats.have_finite_time,
                             stats.min_t_s,
                             stats.max_t_s,
                             stats.max_segment_length_m);
            }
            else
            {
                Logger::info("[KeplerPick] draw={} groups={} group_max={} segments={} requested={} vertices={} "
                             "largest='{}'/{}/{} skipped_pos={} invalid_time={} nonfinite_length={} "
                             "t_valid={} t=[{:.3f},{:.3f}] max_segment_m={:.3f}",
                             stats.draw_call_id,
                             stats.groups,
                             stats.max_group_id,
                             stats.registered_segments,
                             stats.requested_segments,
                             stats.vertices,
                             stats.largest_line_set_owner,
                             stats.largest_line_set_role,
                             stats.largest_line_set_segments,
                             stats.skipped_nonfinite_position_segments,
                             stats.invalid_time_segments,
                             stats.nonfinite_length_segments,
                             stats.have_finite_time,
                             stats.min_t_s,
                             stats.max_t_s,
                             stats.max_segment_length_m);
            }
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

        std::size_t target_pick_segment_count(const KeplerArcLineSet &line_set)
        {
            if (!line_set.valid || line_set.vertices.size() < 2u)
            {
                return 0u;
            }
            return std::min(line_set.vertices.size() - 1u, PickSegmentConfig::max_segments_per_line_set);
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

        bool clip_world_segment_to_view(const KeplerPredictionDrawContext &context,
                                        WorldVec3 &a_world,
                                        WorldVec3 &b_world)
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

            const WorldVec3 original_a = a_world;
            const WorldVec3 original_b = b_world;
            a_world = glm::mix(original_a, original_b, t0);
            b_world = glm::mix(original_a, original_b, t1);
            return finite_world(a_world) && finite_world(b_world);
        }

        double projected_segment_px(const KeplerPredictionDrawContext &context,
                                    const WorldVec3 &a_world,
                                    const WorldVec3 &b_world)
        {
            if (!(context.viewport_width_px > 0.0) ||
                !(context.viewport_height_px > 0.0) ||
                !std::isfinite(context.viewport_width_px) ||
                !std::isfinite(context.viewport_height_px))
            {
                return 0.0;
            }

            const glm::dvec4 clip_a = project_world_to_clip(context, a_world);
            const glm::dvec4 clip_b = project_world_to_clip(context, b_world);
            double t0 = 0.0;
            double t1 = 1.0;
            if (!clip_segment_to_view_t(clip_a, clip_b, t0, t1))
            {
                return 0.0;
            }

            const glm::dvec4 clipped_a = glm::mix(clip_a, clip_b, t0);
            const glm::dvec4 clipped_b = glm::mix(clip_a, clip_b, t1);
            if (!(std::abs(clipped_a.w) > 1.0e-9) ||
                !(std::abs(clipped_b.w) > 1.0e-9) ||
                !std::isfinite(clipped_a.w) ||
                !std::isfinite(clipped_b.w))
            {
                return 0.0;
            }

            const glm::dvec2 ndc_a = glm::dvec2(clipped_a) / clipped_a.w;
            const glm::dvec2 ndc_b = glm::dvec2(clipped_b) / clipped_b.w;
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
            if (!std::isfinite(length_px) || !(length_px > 0.0))
            {
                return 0.0;
            }
            return length_px * dash_period_scale(context);
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
                           const char *line_role,
                           const KeplerArcLineSet &line_set,
                           const glm::vec4 &color,
                           const OrbitPlotDepth depth,
                           const OrbitPlotLineStyle style,
                           const KeplerPredictionDrawContext &draw_context,
                           const float line_overlay_boost,
                           std::vector<Picking::LinePickSegmentData> *pick_segment_scratch,
                           KeplerPickFrameStats &pick_stats)
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

            std::vector<OrbitPlotSystem::LineCommand> lines{};
            const std::size_t segment_count = line_set.vertices.size() - 1u;
            const bool overlay_enabled = line_overlay_boost > 0.0f && color.a > 0.0f;
            lines.reserve(segment_count * (overlay_enabled ? 2u : 1u));
            double dash_cursor_px = 0.0;
            std::size_t skipped_nonfinite_position_segments = 0u;
            std::size_t invalid_time_segments = 0u;
            std::size_t nonfinite_length_segments = 0u;
            std::size_t registered_pick_segments = 0u;
            double min_t_s = 0.0;
            double max_t_s = 0.0;
            double max_segment_length_m = 0.0;
            bool have_finite_time = false;
            const auto record_line_set_time = [&](const double t_s) {
                if (!std::isfinite(t_s))
                {
                    return;
                }
                if (!have_finite_time)
                {
                    min_t_s = t_s;
                    max_t_s = t_s;
                    have_finite_time = true;
                    return;
                }
                min_t_s = std::min(min_t_s, t_s);
                max_t_s = std::max(max_t_s, t_s);
            };
            for (std::size_t i = 1; i < line_set.vertices.size(); ++i)
            {
                const KeplerArcLineVertex &a = line_set.vertices[i - 1u];
                const KeplerArcLineVertex &b = line_set.vertices[i];
                if (!finite_world(a.position_world) || !finite_world(b.position_world))
                {
                    ++skipped_nonfinite_position_segments;
                    continue;
                }

                WorldVec3 draw_a_world = a.position_world;
                WorldVec3 draw_b_world = b.position_world;
                if (!clip_world_segment_to_view(draw_context, draw_a_world, draw_b_world))
                {
                    continue;
                }

                if (pick_enabled)
                {
                    const bool valid_time =
                            std::isfinite(a.t_s) &&
                            std::isfinite(b.t_s) &&
                            b.t_s >= a.t_s;
                    if (!valid_time)
                    {
                        ++invalid_time_segments;
                    }
                    if (std::isfinite(a.t_s))
                    {
                        record_line_set_time(a.t_s);
                    }
                    if (std::isfinite(b.t_s))
                    {
                        record_line_set_time(b.t_s);
                    }

                    const double segment_length_m =
                            glm::length(glm::dvec3(b.position_world - a.position_world));
                    if (std::isfinite(segment_length_m))
                    {
                        max_segment_length_m = std::max(max_segment_length_m, segment_length_m);
                    }
                    else
                    {
                        ++nonfinite_length_segments;
                    }
                }

                float dash_coord_a_px = -1.0f;
                float dash_coord_b_px = -1.0f;
                if (style == OrbitPlotLineStyle::Dashed)
                {
                    const double segment_px = projected_segment_px(draw_context, draw_a_world, draw_b_world);
                    dash_coords_for_segment(dash_cursor_px,
                                            std::isfinite(segment_px) ? segment_px : 0.0,
                                            dash_coord_a_px,
                                            dash_coord_b_px);
                }

                lines.push_back(OrbitPlotSystem::LineCommand{
                        .a_world = draw_a_world,
                        .b_world = draw_b_world,
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
                        lines.push_back(OrbitPlotSystem::LineCommand{
                                .a_world = draw_a_world,
                                .b_world = draw_b_world,
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
                registered_pick_segments =
                        emit_pick_segments(*picking, pick_group, line_set, pick_segment_scratch);
                const std::size_t requested_segments = line_set.vertices.size() - 1u;
                record_pick_frame_line_set(pick_stats,
                                           pick_group,
                                           pick_owner_name,
                                           line_role,
                                           line_set.vertices.size(),
                                           requested_segments,
                                           registered_pick_segments,
                                           skipped_nonfinite_position_segments,
                                           invalid_time_segments,
                                           nonfinite_length_segments,
                                           have_finite_time,
                                           min_t_s,
                                           max_t_s,
                                           max_segment_length_m);
            }
            if (!lines.empty())
            {
                orbit_plot.add_lines(std::span<const OrbitPlotSystem::LineCommand>(lines.data(), lines.size()));
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
                        target_segments += target_pick_segment_count(track.base_lines);
                    }
                    if (context.draw_planned && track.active_player)
                    {
                        target_segments += target_pick_segment_count(track.planned_lines);
                    }
                }
            }

            context.pick_segment_scratch->reserve(target_segments);
        }
    } // namespace

    void draw_kepler_prediction(const KeplerPredictionState &state,
                                const KeplerPredictionDrawContext &context)
    {
        static uint64_t s_draw_call_id = 0u;
        ++s_draw_call_id;

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

        KeplerPickFrameStats pick_stats{};
        pick_stats.draw_call_id = s_draw_call_id;

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
                const std::string base_owner =
                        kepler_orbit_pick_owner(KeplerManeuverOrbitPickRole::Base, track.label);
                const Picking::LinePickPayload base_payload =
                        KeplerManeuverPick::make_payload(KeplerManeuverOrbitPickRole::Base,
                                                          track.entity);
                emit_line_set(*context.orbit_plot,
                              context.picking,
                              context.emit_pick && track.active_player && !track.celestial,
                              base_owner.c_str(),
                              base_payload,
                              "base",
                              track.base_lines,
                              track_base_color(track, context),
                              context.depth,
                              OrbitPlotLineStyle::Solid,
                              context,
                              context.line_overlay_boost,
                              context.pick_segment_scratch,
                              pick_stats);

                if (context.draw_planned && track.active_player)
                {
                    const std::string planned_owner =
                            kepler_orbit_pick_owner(KeplerManeuverOrbitPickRole::Planned, track.label);
                    const Picking::LinePickPayload planned_payload =
                            KeplerManeuverPick::make_payload(KeplerManeuverOrbitPickRole::Planned,
                                                             track.entity);
                    const float planned_overlay_boost =
                            std::max(context.line_overlay_boost, context.planned_line_overlay_boost);
                    emit_line_set(*context.orbit_plot,
                                  context.picking,
                                  context.emit_pick,
                                  planned_owner.c_str(),
                                  planned_payload,
                                  "planned",
                                  track.planned_lines,
                                  context.planned_color,
                                  context.depth,
                                  context.draw_planned_as_dashed ? OrbitPlotLineStyle::Dashed
                                                                 : OrbitPlotLineStyle::Solid,
                                  context,
                                  planned_overlay_boost,
                                  context.pick_segment_scratch,
                                  pick_stats);
                }
            }
            log_pick_frame_stats(pick_stats);
            return;
        }

        log_pick_frame_stats(pick_stats);
    }

    void KeplerPredictionSystem::draw(const KeplerPredictionDrawContext &context) const
    {
        KeplerPredictionDrawContext draw_context = context;
        draw_context.pick_segment_scratch = &_pick_segment_scratch;
        draw_kepler_prediction(_state, draw_context);
    }
} // namespace Game
