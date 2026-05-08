#include "game/states/gameplay/prediction_kepler/kepler_prediction_draw.h"

#include "core/picking/picking_system.h"
#include "core/util/logger.h"
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
        constexpr uint64_t kKeplerPickSummaryLogInterval = 300u;
        constexpr uint64_t kKeplerPickLargeLogInterval = 60u;
        constexpr std::size_t kKeplerPickMaxSegmentsPerLineSet = 4'096u;
        constexpr std::size_t kKeplerPickLargeSegmentCount = 8'192u;
        constexpr std::size_t kKeplerPickWarnSegmentCount = 32'768u;

        bool finite_world(const WorldVec3 &p)
        {
            return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
        }

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
                    stats.registered_segments >= kKeplerPickWarnSegmentCount ||
                    stats.skipped_nonfinite_position_segments > 0u ||
                    stats.invalid_time_segments > 0u ||
                    stats.nonfinite_length_segments > 0u ||
                    group_ids_accumulated;
            const bool periodic_log = (stats.draw_call_id % kKeplerPickSummaryLogInterval) == 1u;
            const bool large_log =
                    stats.registered_segments >= kKeplerPickLargeSegmentCount &&
                    (stats.draw_call_id % kKeplerPickLargeLogInterval) == 1u;
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
                                       const KeplerOrbitLineSet &line_set)
        {
            const std::size_t requested_segments =
                    (line_set.vertices.size() > 1u) ? line_set.vertices.size() - 1u : 0u;
            const std::size_t target_segments =
                    std::min(requested_segments, kKeplerPickMaxSegmentsPerLineSet);
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

                const KeplerOrbitLineVertex &a = line_set.vertices[a_index];
                const KeplerOrbitLineVertex &b = line_set.vertices[b_index];
                if (!finite_world(a.position_world) || !finite_world(b.position_world))
                {
                    return;
                }

                picking.add_line_pick_segment(pick_group,
                                              a.position_world,
                                              b.position_world,
                                              a.t_s,
                                              b.t_s);
                ++emitted_segments;
            };

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

            return emitted_segments;
        }

        void emit_line_set(OrbitPlotSystem &orbit_plot,
                           PickingSystem *picking,
                           const bool emit_pick,
                           const char *pick_owner_name,
                           const char *line_role,
                           const KeplerOrbitLineSet &line_set,
                           const glm::vec4 &color,
                           const OrbitPlotDepth depth,
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
                pick_group = picking->add_line_pick_group(pick_owner_name);
            }

            std::vector<OrbitPlotSystem::LineCommand> lines{};
            lines.reserve(line_set.vertices.size() - 1u);
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
                const KeplerOrbitLineVertex &a = line_set.vertices[i - 1u];
                const KeplerOrbitLineVertex &b = line_set.vertices[i];
                if (!finite_world(a.position_world) || !finite_world(b.position_world))
                {
                    ++skipped_nonfinite_position_segments;
                    continue;
                }

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

                lines.push_back(OrbitPlotSystem::LineCommand{
                        .a_world = a.position_world,
                        .b_world = b.position_world,
                        .color = color,
                        .depth = depth,
                });
            }
            if (pick_enabled)
            {
                registered_pick_segments = emit_pick_segments(*picking, pick_group, line_set);
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
            const float alpha =
                    track.celestial_nbody ? 0.42f : (track.celestial ? 0.58f : context.base_color.a);
            return glm::vec4(track.orbit_rgb, alpha);
        }
    } // namespace

    void draw_kepler_prediction(const KeplerPredictionState &state,
                                const KeplerPredictionDrawContext &context)
    {
        static uint64_t s_draw_call_id = 0u;
        ++s_draw_call_id;

        if (context.picking)
        {
            context.picking->clear_line_picks();
            context.picking->settings().enable_line_hover = context.emit_pick && state.enabled;
        }
        if (context.orbit_plot)
        {
            context.orbit_plot->clear_pending();
        }

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

                emit_line_set(*context.orbit_plot,
                              context.picking,
                              context.emit_pick && !track.celestial,
                              track.label.empty() ? "KeplerOrbit/Base" : track.label.c_str(),
                              "base",
                              track.base_lines,
                              track_base_color(track, context),
                              context.depth,
                              pick_stats);

                if (context.draw_planned && track.active_player)
                {
                    emit_line_set(*context.orbit_plot,
                                  context.picking,
                                  context.emit_pick,
                                  track.label.empty() ? "KeplerOrbit/Planned" : track.label.c_str(),
                                  "planned",
                                  track.planned_lines,
                                  context.planned_color,
                                  context.depth,
                                  pick_stats);
                }
            }
            log_pick_frame_stats(pick_stats);
            return;
        }

        emit_line_set(*context.orbit_plot,
                      context.picking,
                      context.emit_pick,
                      "KeplerOrbit/Base",
                      "base",
                      state.base_lines,
                      context.base_color,
                      context.depth,
                      pick_stats);

        if (context.draw_planned)
        {
            emit_line_set(*context.orbit_plot,
                          context.picking,
                          context.emit_pick,
                          "KeplerOrbit/Planned",
                          "planned",
                          state.planned_lines,
                          context.planned_color,
                          context.depth,
                          pick_stats);
        }

        log_pick_frame_stats(pick_stats);
    }

    void KeplerPredictionSystem::draw(const KeplerPredictionDrawContext &context) const
    {
        draw_kepler_prediction(_state, context);
    }
} // namespace Game
