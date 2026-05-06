#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/prediction/gameplay_prediction_adapter.h"
#include "game/states/gameplay/prediction/draw/gameplay_state_prediction_draw_internal.h"
#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"
#include "game/orbit/orbit_prediction_tuning.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace Game
{
    namespace Draw = PredictionDrawDetail;
    namespace
    {
        constexpr double kPlannedAdaptiveSelectionErrorScale = 0.05;
        constexpr std::size_t kPlannedDashFocusOnlyNodeThreshold = 24u;
        constexpr double kPlannedDashFocusMinWindowS = 60.0;
        constexpr double kPlannedDashFocusMaxWindowS = OrbitPredictionTuning::kPredictionChunkSpanNearS;
    }

    Draw::PredictionRenderEmitter::PredictionRenderEmitter(GameplayPredictionAdapter &adapter)
        : _adapter(adapter)
    {
    }

    void Draw::PredictionRenderEmitter::emit(Draw::PredictionTrackVisualPlan &plan)
    {
        Draw::PredictionTrackDrawContext &track_ctx = plan.track;
        GameplayPredictionState &prediction_state = _adapter.prediction_draw_state();
        const GameplayPredictionContext adapter_context = _adapter.context();
        const ManeuverPlanState &maneuver_plan = adapter_context.maneuver.plan();
        OrbitPredictionCache &stable_cache = *track_ctx.stable_cache;
        OrbitPredictionCache &planned_cache = *track_ctx.planned_cache;
        int planned_dash_chunks_remaining = std::max(0, prediction_state.draw_config.dash_max_chunks_per_window);
        const double planned_draw_span_s =
                track_ctx.planned_draw_window.valid
                        ? (track_ctx.planned_draw_window.t1_s - track_ctx.planned_draw_window.t0_s)
                        : 0.0;
        const bool long_or_dense_planned_plan =
                maneuver_plan.nodes.size() > kPlannedDashFocusOnlyNodeThreshold ||
                (std::isfinite(planned_draw_span_s) &&
                 planned_draw_span_s > kPlannedDashFocusMaxWindowS);
        const bool disable_planned_overlay_boost =
                track_ctx.active_player_track &&
                long_or_dense_planned_plan;
        const bool interactive_maneuver_edit_active =
                track_ctx.maneuver_drag_active ||
                adapter_context.maneuver.edit_preview().state != ManeuverNodeEditPreview::State::Idle;
        const bool planned_render_line_cache_allowed =
                !interactive_maneuver_edit_active &&
                plan.lifecycle.state == PredictionTrackLifecycleState::Stable;

        const auto resolve_planned_dash_focus_window = [&]() {
            Draw::PickWindow window{};
            if (!track_ctx.planned_draw_window.valid)
            {
                return window;
            }

            double anchor_time_s = std::numeric_limits<double>::quiet_NaN();
            double focus_window_s = std::numeric_limits<double>::quiet_NaN();
            if (track_ctx.track &&
                track_ctx.track->preview_anchor.valid &&
                std::isfinite(track_ctx.track->preview_anchor.anchor_time_s))
            {
                anchor_time_s = track_ctx.track->preview_anchor.anchor_time_s;
                focus_window_s = track_ctx.track->preview_anchor.visual_window_s;
            }

            if (!std::isfinite(anchor_time_s))
            {
                if (const ManeuverNode *anchor_node =
                            maneuver_plan.find_node(adapter_context.maneuver.active_preview_anchor_node_id()))
                {
                    anchor_time_s = anchor_node->time_s;
                }
            }
            if (!std::isfinite(anchor_time_s))
            {
                if (const ManeuverNode *selected_node = maneuver_plan.find_node(maneuver_plan.selected_node_id))
                {
                    anchor_time_s = selected_node->time_s;
                }
            }
            if (!std::isfinite(anchor_time_s))
            {
                anchor_time_s = track_ctx.planned_draw_window.anchor_time_s;
            }
            if (!std::isfinite(anchor_time_s))
            {
                return window;
            }

            if (!std::isfinite(focus_window_s) || !(focus_window_s > 0.0))
            {
                focus_window_s = track_ctx.planned_exact_window_s;
            }
            if (!std::isfinite(focus_window_s) || !(focus_window_s > 0.0))
            {
                focus_window_s = track_ctx.planned_visual_window_s;
            }
            if (!std::isfinite(focus_window_s) || !(focus_window_s > 0.0))
            {
                focus_window_s = kPlannedDashFocusMaxWindowS;
            }
            focus_window_s = std::clamp(focus_window_s,
                                        kPlannedDashFocusMinWindowS,
                                        kPlannedDashFocusMaxWindowS);

            window.valid = true;
            if (anchor_time_s >= track_ctx.planned_draw_window.t1_s)
            {
                window.t1_s = track_ctx.planned_draw_window.t1_s;
                window.t0_s = std::max(track_ctx.planned_draw_window.t0_s, window.t1_s - focus_window_s);
            }
            else
            {
                window.t0_s = std::clamp(anchor_time_s,
                                         track_ctx.planned_draw_window.t0_s,
                                         track_ctx.planned_draw_window.t1_s);
                window.t1_s = std::min(track_ctx.planned_draw_window.t1_s, window.t0_s + focus_window_s);
            }
            window.anchor_time_s = anchor_time_s;
            if (!(window.t1_s > window.t0_s))
            {
                window = {};
            }
            return window;
        };
        const Draw::PickWindow planned_dash_focus_window = resolve_planned_dash_focus_window();
        const bool planned_dash_focus_only =
                long_or_dense_planned_plan &&
                planned_dash_focus_window.valid;
        const auto planned_draw_context_for = [&](const Draw::OrbitDrawWindowContext &base_ctx,
                                                  const bool dashed) {
            Draw::OrbitDrawWindowContext out = base_ctx;
            if (disable_planned_overlay_boost)
            {
                out.line_overlay_boost = 0.0f;
            }
            if (dashed && planned_dash_chunks_remaining > 0)
            {
                out.dash_chunks_remaining = &planned_dash_chunks_remaining;
            }
            else
            {
                out.dash_chunks_remaining = nullptr;
            }
            return out;
        };
        const auto draw_planned_split_ranges = [&](const double window_t0_s,
                                                   const double window_t1_s,
                                                   const bool allow_dashed,
                                                   const auto &draw_range) {
            bool drew = false;
            const auto draw_one = [&](const double range_t0_s,
                                      const double range_t1_s,
                                      const bool dashed) {
                if (!(range_t1_s > range_t0_s))
                {
                    return;
                }
                drew = draw_range(range_t0_s,
                                  range_t1_s,
                                  dashed && planned_dash_chunks_remaining > 0) || drew;
            };

            if (!allow_dashed || planned_dash_chunks_remaining <= 0)
            {
                draw_one(window_t0_s, window_t1_s, false);
                return drew;
            }
            if (!planned_dash_focus_only)
            {
                draw_one(window_t0_s, window_t1_s, true);
                return drew;
            }

            const double dash_t0_s = std::max(window_t0_s, planned_dash_focus_window.t0_s);
            const double dash_t1_s = std::min(window_t1_s, planned_dash_focus_window.t1_s);
            if (!(dash_t1_s > dash_t0_s))
            {
                draw_one(window_t0_s, window_t1_s, false);
                return drew;
            }

            draw_one(window_t0_s, dash_t0_s, false);
            draw_one(dash_t0_s, dash_t1_s, true);
            draw_one(dash_t1_s, window_t1_s, false);
            return drew;
        };
        const auto draw_raw_base_window = [&](const double split_t0_s,
                                              const double split_t1_s,
                                              const glm::vec4 &color) {
            if (!(split_t1_s > split_t0_s))
            {
                return;
            }
            Draw::draw_orbit_window(track_ctx.identity_frame_transform ? track_ctx.draw_ctx : track_ctx.world_basis_draw_ctx,
                                    prediction_state.draw_config,
                                    prediction_state.orbit_plot_perf,
                                    track_ctx.identity_frame_transform ? *track_ctx.traj_base_segments
                                                                       : Draw::base_segments_world_basis(track_ctx),
                                    split_t0_s,
                                    split_t1_s,
                                    color,
                                    false);
        };

        const auto draw_cpu_base_window = [&](const double window_t0_s,
                                              const double window_t1_s,
                                              const glm::vec4 &color) {
            if (!(window_t1_s > window_t0_s))
            {
                return;
            }
            const auto draw_cpu_base_window_once = [&](const double split_t0_s, const double split_t1_s) {
                if (!(split_t1_s > split_t0_s))
                {
                    return;
                }
                if (!track_ctx.use_base_adaptive_curve)
                {
                    draw_raw_base_window(split_t0_s, split_t1_s, color);
                    return;
                }
                std::size_t anchor_hi = track_ctx.i_hi;
                if (anchor_hi >= track_ctx.traj_base->size())
                {
                    anchor_hi = track_ctx.traj_base->size() - 1;
                }
                std::size_t anchor_lo = (anchor_hi > 0) ? (anchor_hi - 1) : 0;
                if (anchor_hi == anchor_lo && (anchor_hi + 1) < track_ctx.traj_base->size())
                {
                    anchor_hi = anchor_lo + 1;
                }
                const double anchor_source_t0_s = (*track_ctx.traj_base)[anchor_lo].t_s;
                const double anchor_source_t1_s = (*track_ctx.traj_base)[anchor_hi].t_s;
                const double anchor_t0_s = std::max(split_t0_s, anchor_source_t0_s);
                const double anchor_t1_s = std::min(split_t1_s, anchor_source_t1_s);
                const bool anchor_window_valid =
                        track_ctx.is_active &&
                        std::isfinite(anchor_source_t0_s) &&
                        std::isfinite(anchor_source_t1_s) &&
                        (anchor_t1_s > anchor_t0_s);

                if (!anchor_window_valid)
                {
                    Draw::draw_adaptive_curve_window(track_ctx.draw_ctx,
                                                     prediction_state.draw_config,
                                                     prediction_state.orbit_plot_perf,
                                                     stable_cache.display.render_curve_frame,
                                                     split_t0_s,
                                                     split_t1_s,
                                                     color,
                                                     false);
                    return;
                }

                if (anchor_t0_s > split_t0_s)
                {
                    Draw::draw_adaptive_curve_window(track_ctx.draw_ctx,
                                                     prediction_state.draw_config,
                                                     prediction_state.orbit_plot_perf,
                                                     stable_cache.display.render_curve_frame,
                                                     split_t0_s,
                                                     anchor_t0_s,
                                                     color,
                                                     false);
                }

                draw_raw_base_window(anchor_t0_s, anchor_t1_s, color);

                if (split_t1_s > anchor_t1_s)
                {
                    Draw::draw_adaptive_curve_window(track_ctx.draw_ctx,
                                                     prediction_state.draw_config,
                                                     prediction_state.orbit_plot_perf,
                                                     stable_cache.display.render_curve_frame,
                                                     anchor_t1_s,
                                                     split_t1_s,
                                                     color,
                                                     false);
                }
            };

            if (track_ctx.now_s > window_t0_s && track_ctx.now_s < window_t1_s)
            {
                draw_cpu_base_window_once(window_t0_s, track_ctx.now_s);
                draw_cpu_base_window_once(track_ctx.now_s, window_t1_s);
                return;
            }
            draw_cpu_base_window_once(window_t0_s, window_t1_s);
        };

        const auto planned_cache_drawable = [&](const OrbitPredictionCache &cache) {
            const bool primary_cache_enabled =
                    &cache == track_ctx.planned_cache &&
                    track_ctx.planned_cache_drawable;
            const bool stale_cache_enabled =
                    &cache == track_ctx.stale_planned_cache &&
                    track_ctx.stale_planned_cache_drawable;
            if (!primary_cache_enabled && !stale_cache_enabled)
            {
                return false;
            }

            return cache.display.planned_chunk_assembly.drawable();
        };

        PredictionTimeAnchorCache local_preview_time_cache{};
        PredictionRenderLinePacketCache local_planned_render_line_cache{};
        PredictionRenderLinePacketCache local_stale_planned_render_line_cache{};
        const auto preview_time_cache_for = [&](const OrbitPredictionCache &cache) -> PredictionTimeAnchorCache & {
            if (!track_ctx.track)
            {
                return local_preview_time_cache;
            }
            return (&cache == track_ctx.stale_planned_cache)
                           ? track_ctx.track->stale_planned_curve_preview_time_cache
                           : track_ctx.track->planned_curve_preview_time_cache;
        };
        const auto render_line_cache_for = [&](const OrbitPredictionCache &cache) -> PredictionRenderLinePacketCache & {
            if (!track_ctx.track)
            {
                return (&cache == track_ctx.stale_planned_cache)
                               ? local_stale_planned_render_line_cache
                               : local_planned_render_line_cache;
            }
            return (&cache == track_ctx.stale_planned_cache)
                           ? track_ctx.track->stale_planned_render_line_cache
                           : track_ctx.track->planned_render_line_cache;
        };
        const auto collect_planned_curve_anchor_times = [&](OrbitPredictionCache &cache,
                                                            const double window_t0_s,
                                                            const double window_t1_s) {
            const bool preview_anchor_valid = track_ctx.track && track_ctx.track->preview_anchor.valid;
            const double preview_anchor_time_s =
                    preview_anchor_valid
                            ? track_ctx.track->preview_anchor.anchor_time_s
                            : std::numeric_limits<double>::quiet_NaN();
            return Draw::collect_planned_curve_anchor_times(prediction_state.maneuver_node_time_cache,
                                                            preview_time_cache_for(cache),
                                                            maneuver_plan.nodes,
                                                            adapter_context.maneuver.revision(),
                                                            cache,
                                                            preview_anchor_time_s,
                                                            preview_anchor_valid,
                                                            window_t0_s,
                                                            window_t1_s);
        };

        const auto draw_planned_window_from_cache = [&](OrbitPredictionCache &cache,
                                                        const double window_t0_s,
                                                        const double window_t1_s,
                                                         const glm::vec4 &color,
                                                         const bool force_solid = false,
                                                         const bool ignore_prefix_clip = false) {
            double clipped_window_t1_s = window_t1_s;
            const bool drawing_stale_cache =
                    &cache == track_ctx.stale_planned_cache &&
                    track_ctx.stale_planned_cache_drawable;
            if (drawing_stale_cache)
            {
                if (!std::isfinite(track_ctx.stale_planned_cache_prefix_cutoff_s))
                {
                    return;
                }
                clipped_window_t1_s = std::min(clipped_window_t1_s,
                                               track_ctx.stale_planned_cache_prefix_cutoff_s);
            }
            else if (track_ctx.planned_cache_prefix_only && !ignore_prefix_clip)
            {
                if (!std::isfinite(track_ctx.planned_cache_prefix_cutoff_s))
                {
                    return;
                }
                clipped_window_t1_s = std::min(clipped_window_t1_s, track_ctx.planned_cache_prefix_cutoff_s);
            }
            if (!(clipped_window_t1_s > window_t0_s) || !planned_cache_drawable(cache))
            {
                return;
            }
            const bool allow_dashed =
                    prediction_state.draw_config.draw_planned_as_dashed &&
                    !force_solid &&
                    !track_ctx.maneuver_drag_active &&
                    planned_dash_chunks_remaining > 0;
            const auto draw_range = [&](const double range_t0_s,
                                        const double range_t1_s,
                                        const bool dashed) {
                bool drew = false;
                for (const OrbitChunk &chunk : cache.display.planned_chunk_assembly.chunks)
                {
                    const double chunk_t0_s = std::max(range_t0_s, chunk.t0_s);
                    const double chunk_t1_s = std::min(range_t1_s, chunk.t1_s);
                    if (!(chunk_t1_s > chunk_t0_s) ||
                        !chunk.drawable())
                    {
                        continue;
                    }

                    const Draw::OrbitDrawWindowContext planned_ctx =
                            planned_draw_context_for(track_ctx.draw_ctx, dashed);
                    if (track_ctx.direct_world_polyline && chunk.frame_samples.size() >= 2)
                    {
                        Draw::draw_polyline_window(planned_ctx,
                                                   prediction_state.draw_config,
                                                   chunk.frame_samples,
                                                   chunk_t0_s,
                                                   chunk_t1_s,
                                                   color,
                                                   dashed);
                        drew = true;
                        continue;
                    }

                    if (!chunk.render_curve.empty())
                    {
                        const std::vector<double> anchors =
                                collect_planned_curve_anchor_times(cache, chunk_t0_s, chunk_t1_s);
                        Draw::draw_adaptive_curve_window(planned_ctx,
                                                         prediction_state.draw_config,
                                                         prediction_state.orbit_plot_perf,
                                                         chunk.render_curve,
                                                         chunk_t0_s,
                                                         chunk_t1_s,
                                                         color,
                                                         dashed,
                                                         anchors,
                                                         kPlannedAdaptiveSelectionErrorScale);
                        drew = true;
                        continue;
                    }

                    if (!chunk.frame_segments.empty())
                    {
                        Draw::draw_orbit_window(planned_ctx,
                                                prediction_state.draw_config,
                                                prediction_state.orbit_plot_perf,
                                                chunk.frame_segments,
                                                chunk_t0_s,
                                                chunk_t1_s,
                                                color,
                                                dashed);
                        drew = true;
                    }
                }
                return drew;
            };
            (void) draw_planned_split_ranges(window_t0_s,
                                             clipped_window_t1_s,
                                             allow_dashed,
                                             draw_range);
        };

        const auto chunk_drawable = [&](const OrbitChunk &chunk) {
            if (track_ctx.direct_world_polyline)
            {
                return chunk.frame_samples.size() >= 2 ||
                       !chunk.render_curve.empty() ||
                       !chunk.frame_segments.empty();
            }

            return !chunk.render_curve.empty() || !chunk.frame_segments.empty();
        };

        const auto draw_planned_window_from_chunk = [&](const OrbitChunk &chunk,
                                                        const double window_t0_s,
                                                        const double window_t1_s,
                                                        const glm::vec4 &color,
                                                        const bool force_solid = false) {
            if (!(window_t1_s > window_t0_s) || !chunk_drawable(chunk))
            {
                return false;
            }
            const bool allow_dashed =
                    prediction_state.draw_config.draw_planned_as_dashed &&
                    !force_solid &&
                    !track_ctx.maneuver_drag_active &&
                    planned_dash_chunks_remaining > 0;

            const auto draw_range = [&](const double range_t0_s,
                                        const double range_t1_s,
                                        const bool dashed) {
                const Draw::OrbitDrawWindowContext planned_ctx =
                        planned_draw_context_for(track_ctx.draw_ctx, dashed);
                if (track_ctx.direct_world_polyline && chunk.frame_samples.size() >= 2)
                {
                    Draw::draw_polyline_window(planned_ctx,
                                               prediction_state.draw_config,
                                               chunk.frame_samples,
                                               range_t0_s,
                                               range_t1_s,
                                               color,
                                               dashed);
                    return true;
                }

                if (!chunk.render_curve.empty())
                {
                    Draw::draw_adaptive_curve_window(planned_ctx,
                                                     prediction_state.draw_config,
                                                     prediction_state.orbit_plot_perf,
                                                     chunk.render_curve,
                                                     range_t0_s,
                                                     range_t1_s,
                                                     color,
                                                     dashed,
                                                     std::span<const double>{},
                                                     kPlannedAdaptiveSelectionErrorScale);
                    return true;
                }

                if (chunk.frame_segments.empty())
                {
                    return false;
                }

                Draw::draw_orbit_window(planned_ctx,
                                        prediction_state.draw_config,
                                        prediction_state.orbit_plot_perf,
                                        chunk.frame_segments,
                                        range_t0_s,
                                        range_t1_s,
                                        color,
                                        dashed);
                return true;
            };
            return draw_planned_split_ranges(window_t0_s,
                                             window_t1_s,
                                             allow_dashed,
                                             draw_range);
        };

        const auto prefix_fallback_cache = [&]() -> OrbitPredictionCache * {
            if (track_ctx.stale_planned_cache && track_ctx.stale_planned_cache_drawable)
            {
                return track_ctx.stale_planned_cache;
            }
            if (planned_cache_drawable(planned_cache))
            {
                return &planned_cache;
            }
            return nullptr;
        };

        const auto clamp_prefix_cutoff_for_cache = [&](OrbitPredictionCache *cache,
                                                       const double requested_cutoff_s) {
            double cutoff_s = requested_cutoff_s;
            if (cache == track_ctx.stale_planned_cache &&
                std::isfinite(track_ctx.stale_planned_cache_prefix_cutoff_s))
            {
                cutoff_s = std::isfinite(cutoff_s)
                                   ? std::min(cutoff_s, track_ctx.stale_planned_cache_prefix_cutoff_s)
                                   : track_ctx.stale_planned_cache_prefix_cutoff_s;
            }
            return cutoff_s;
        };

        if (plan.base_full_draw_window.valid)
        {
            if (track_ctx.direct_world_polyline)
            {
                Draw::draw_polyline_window(track_ctx.draw_ctx,
                                           prediction_state.draw_config,
                                           *track_ctx.traj_base,
                                           plan.base_full_draw_window.t0_s,
                                           plan.base_full_draw_window.t1_s,
                                           track_ctx.track_color_full,
                                           false);
            }
            else
            {
                draw_cpu_base_window(plan.base_full_draw_window.t0_s,
                                     plan.base_full_draw_window.t1_s,
                                     track_ctx.track_color_full);
            }
        }

        if (plan.base_future_draw_window.valid)
        {
            if (track_ctx.direct_world_polyline)
            {
                Draw::draw_polyline_window(track_ctx.draw_ctx,
                                           prediction_state.draw_config,
                                           *track_ctx.traj_base,
                                           plan.base_future_draw_window.t0_s,
                                           plan.base_future_draw_window.t1_s,
                                           track_ctx.track_color_future,
                                           false);
            }
            else
            {
                draw_cpu_base_window(plan.base_future_draw_window.t0_s,
                                     plan.base_future_draw_window.t1_s,
                                     track_ctx.track_color_future);
            }
        }

        if (track_ctx.active_player_track)
        {
            prediction_state.orbit_plot_perf.planned_window_valid = track_ctx.planned_draw_window.valid;
            prediction_state.orbit_plot_perf.planned_window_now_s = track_ctx.now_s;
            prediction_state.orbit_plot_perf.planned_window_anchor_s = track_ctx.planned_draw_window.anchor_time_s;
            prediction_state.orbit_plot_perf.planned_window_t_start = track_ctx.planned_draw_window.t0_s;
            prediction_state.orbit_plot_perf.planned_window_t_end = track_ctx.planned_draw_window.t1_s;
            double planned_t0_s = 0.0;
            double planned_t1_s = 0.0;
            prediction_state.orbit_plot_perf.planned_window_t0p =
                    (track_ctx.planned_window_assembly &&
                     track_ctx.planned_window_assembly->time_span(planned_t0_s, planned_t1_s))
                            ? planned_t0_s
                            : 0.0;
            prediction_state.orbit_plot_perf.planned_chunk_count = 0;
            prediction_state.orbit_plot_perf.planned_chunks_drawn = 0;
            prediction_state.orbit_plot_perf.planned_chunk_enqueue_ms_last = 0.0;
            prediction_state.orbit_plot_perf.planned_fallback_range_count = 0;
            prediction_state.orbit_plot_perf.planned_fallback_draw_ms_last = 0.0;
        }

        if (!track_ctx.planned_draw_window.valid)
        {
            return;
        }

        const double planned_window_t0_s = track_ctx.planned_draw_window.t0_s;
        const double planned_window_t1_s = track_ctx.planned_draw_window.t1_s;
        const glm::vec4 preview_plan_color = plan.preview_plan_color;
        const PredictionChunkAssembly &preview_assembly = plan.preview_assembly;
        const PredictionChunkAssembly *full_stream_assembly =
                plan.full_stream_overlay_active
                        ? &plan.full_stream_assembly
                        : nullptr;
        const auto draw_chunk_assembly_ranges =
                [&](const PredictionChunkAssembly &assembly,
                    const glm::vec4 &color,
                    const std::vector<std::pair<double, double>> *masked_ranges,
                    std::vector<std::pair<double, double>> &out_drawn_ranges) {
                    prediction_state.orbit_plot_perf.planned_chunk_count += static_cast<uint32_t>(assembly.chunks.size());
                    for (const OrbitChunk &chunk : assembly.chunks)
                    {
                        if (!std::isfinite(chunk.t0_s) || !std::isfinite(chunk.t1_s))
                        {
                            continue;
                        }
                        const double clipped_t0_s = std::max(planned_window_t0_s, chunk.t0_s);
                        const double clipped_t1_s = std::min(planned_window_t1_s, chunk.t1_s);
                        if (!std::isfinite(clipped_t0_s) ||
                            !std::isfinite(clipped_t1_s) ||
                            !(clipped_t1_s > clipped_t0_s))
                        {
                            continue;
                        }

                        std::vector<std::pair<double, double>> draw_ranges;
                        if (masked_ranges && !masked_ranges->empty())
                        {
                            draw_ranges = Draw::compute_uncovered_ranges(clipped_t0_s, clipped_t1_s, *masked_ranges);
                        }
                        else
                        {
                            draw_ranges.emplace_back(clipped_t0_s, clipped_t1_s);
                        }

                        bool drew_chunk = false;
                        for (const auto &[range_t0_s, range_t1_s] : draw_ranges)
                        {
                            if (!draw_planned_window_from_chunk(chunk,
                                                                range_t0_s,
                                                                range_t1_s,
                                                                color,
                                                                track_ctx.maneuver_drag_active))
                            {
                                continue;
                            }

                            out_drawn_ranges.emplace_back(range_t0_s, range_t1_s);
                            drew_chunk = true;
                        }

                        if (drew_chunk)
                        {
                            ++prediction_state.orbit_plot_perf.planned_chunks_drawn;
                        }
                    }
                };
        const auto draw_cache_fallback_ranges =
                [&](const std::vector<std::pair<double, double>> &covered_ranges,
                    const double fresh_cutoff_s) {
                    OrbitPredictionCache *fallback_cache = prefix_fallback_cache();
                    if (!fallback_cache || !planned_cache_drawable(*fallback_cache))
                    {
                        return;
                    }
                    const double prefix_cutoff_s =
                            clamp_prefix_cutoff_for_cache(fallback_cache, fresh_cutoff_s);

                    const std::vector<std::pair<double, double>> uncovered_ranges =
                            Draw::compute_uncovered_ranges(planned_window_t0_s,
                                                           planned_window_t1_s,
                                                           covered_ranges);
                    prediction_state.orbit_plot_perf.planned_fallback_range_count =
                            static_cast<uint32_t>(uncovered_ranges.size());
                    for (const auto &[range_t0_s, range_t1_s] : uncovered_ranges)
                    {
                        const bool prefix_range =
                                std::isfinite(prefix_cutoff_s) &&
                                range_t1_s <= (prefix_cutoff_s + 1.0e-6);
                        if (!prefix_range)
                        {
                            continue;
                        }
                        draw_planned_window_from_cache(*fallback_cache,
                                                       range_t0_s,
                                                       range_t1_s,
                                                       track_ctx.track_color_plan);
                    }
                };
        const auto sample_chunk_assembly_world = [&](const PredictionChunkAssembly &assembly,
                                                     const double sample_t_s,
                                                     WorldVec3 &out_world) {
            for (const OrbitChunk &chunk : assembly.chunks)
            {
                if (!chunk.drawable() ||
                    !std::isfinite(chunk.t0_s) ||
                    !std::isfinite(chunk.t1_s) ||
                    sample_t_s < chunk.t0_s ||
                    sample_t_s > chunk.t1_s)
                {
                    continue;
                }
                if (Draw::sample_prediction_path_world(track_ctx.draw_ctx,
                                                       chunk.frame_segments,
                                                       chunk.frame_samples,
                                                       sample_t_s,
                                                       out_world))
                {
                    return true;
                }
            }
            return false;
        };
        const auto preview_tail_matches_planned_cache = [&]() {
            if (!preview_assembly.valid || preview_assembly.chunks.empty())
            {
                return false;
            }
            if (!planned_cache.display.planned_chunk_assembly.drawable())
            {
                return false;
            }

            constexpr std::size_t kSamplesPerChunk = 5u;
            constexpr std::size_t kMaxTotalSamples = 24u;
            constexpr double kMaxPointErrorPx = 1.75;
            constexpr double kMaxAverageErrorPx = 0.85;

            double total_error_px = 0.0;
            std::size_t sample_count = 0u;
            for (const OrbitChunk &chunk : preview_assembly.chunks)
            {
                const double clipped_t0_s = std::max(planned_window_t0_s, chunk.t0_s);
                const double clipped_t1_s = std::min(planned_window_t1_s, chunk.t1_s);
                if (!(clipped_t1_s > clipped_t0_s))
                {
                    continue;
                }

                const std::size_t samples_this_chunk =
                        std::min<std::size_t>(kSamplesPerChunk, kMaxTotalSamples - sample_count);
                if (samples_this_chunk == 0u)
                {
                    break;
                }

                for (std::size_t i = 0; i < samples_this_chunk; ++i)
                {
                    const double u =
                            (samples_this_chunk > 1u) ? static_cast<double>(i) / static_cast<double>(samples_this_chunk - 1u)
                                                      : 0.5;
                    const double sample_t_s = clipped_t0_s + ((clipped_t1_s - clipped_t0_s) * u);

                    WorldVec3 preview_world{0.0};
                    WorldVec3 planned_world{0.0};
                    if (!Draw::sample_prediction_path_world(track_ctx.draw_ctx,
                                                            chunk.frame_segments,
                                                              chunk.frame_samples,
                                                              sample_t_s,
                                                              preview_world) ||
                        !sample_chunk_assembly_world(planned_cache.display.planned_chunk_assembly,
                                                     sample_t_s,
                                                     planned_world))
                    {
                        continue;
                    }

                    const double error_m = glm::length(glm::dvec3(preview_world - planned_world));
                    const WorldVec3 error_mid_world =
                            WorldVec3(glm::mix(glm::dvec3(preview_world), glm::dvec3(planned_world), 0.5));
                    const double meters_per_px =
                            std::max(1.0e-6, Draw::meters_per_px_at_world(track_ctx.draw_ctx, error_mid_world));
                    const double error_px = error_m / meters_per_px;
                    if (!std::isfinite(error_px) || error_px > kMaxPointErrorPx)
                    {
                        return false;
                    }

                    total_error_px += error_px;
                    ++sample_count;
                }
            }

            return sample_count > 0u &&
                   (total_error_px / static_cast<double>(sample_count)) <= kMaxAverageErrorPx;
        };
        const bool draw_matching_cached_tail = preview_tail_matches_planned_cache();
        if (!preview_assembly.valid || preview_assembly.chunks.empty())
        {
            const bool preview_fallback_active =
                    track_ctx.track->preview_anchor.valid &&
                    plan.overlay_layers.preview_fallback_active &&
                    std::isfinite(track_ctx.track->preview_anchor.anchor_time_s) &&
                    track_ctx.track->preview_anchor.visual_window_s > 0.0;
            if (preview_fallback_active)
            {
                const double preview_t0_s = std::clamp(track_ctx.track->preview_anchor.anchor_time_s,
                                                       planned_window_t0_s,
                                                       planned_window_t1_s);
                const double preview_t1_s =
                        std::min(planned_window_t1_s,
                                 preview_t0_s + track_ctx.track->preview_anchor.visual_window_s);
                if (preview_t0_s > planned_window_t0_s)
                {
                    if (OrbitPredictionCache *fallback_cache = prefix_fallback_cache())
                    {
                        draw_planned_window_from_cache(*fallback_cache,
                                                       planned_window_t0_s,
                                                       preview_t0_s,
                                                       track_ctx.track_color_plan);
                    }
                }
                if (preview_t1_s > preview_t0_s && planned_cache_drawable(planned_cache))
                {
                    draw_planned_window_from_cache(planned_cache,
                                                   preview_t0_s,
                                                   preview_t1_s,
                                                   preview_plan_color,
                                                   track_ctx.maneuver_drag_active,
                                                   true);
                }
                return;
            }
            if (full_stream_assembly)
            {
                std::vector<std::pair<double, double>> full_stream_covered_ranges;
                full_stream_covered_ranges.reserve(full_stream_assembly->chunks.size());
                draw_chunk_assembly_ranges(
                        *full_stream_assembly,
                        track_ctx.track_color_plan,
                        nullptr,
                        full_stream_covered_ranges);
                draw_cache_fallback_ranges(full_stream_covered_ranges, planned_window_t1_s);
                return;
            }
            if (track_ctx.maneuver_drag_active && std::isfinite(track_ctx.planned_draw_window.anchor_time_s))
            {
                if (OrbitPredictionCache *fallback_cache = prefix_fallback_cache())
                {
                    draw_planned_window_from_cache(*fallback_cache,
                                                   planned_window_t0_s,
                                                   std::min(planned_window_t1_s,
                                                            track_ctx.planned_draw_window.anchor_time_s),
                                                   track_ctx.track_color_plan);
                }
                return;
            }
            if (OrbitPredictionCache *fallback_cache = prefix_fallback_cache())
            {
                draw_planned_window_from_cache(*fallback_cache,
                                               planned_window_t0_s,
                                               planned_window_t1_s,
                                               track_ctx.track_color_plan);
            }
            return;
        }

        std::vector<std::pair<double, double>> covered_ranges;
        covered_ranges.reserve(preview_assembly.chunks.size());
        double first_preview_t0_s = std::numeric_limits<double>::quiet_NaN();
        prediction_state.orbit_plot_perf.planned_chunk_count += static_cast<uint32_t>(preview_assembly.chunks.size());
        for (const OrbitChunk &chunk : preview_assembly.chunks)
        {
            const double clipped_t0_s = std::max(planned_window_t0_s, chunk.t0_s);
            const double clipped_t1_s = std::min(planned_window_t1_s, chunk.t1_s);
            if (!(clipped_t1_s > clipped_t0_s))
            {
                continue;
            }

            if (!draw_planned_window_from_chunk(chunk,
                                                clipped_t0_s,
                                                clipped_t1_s,
                                                preview_plan_color,
                                                track_ctx.maneuver_drag_active))
            {
                continue;
            }

            covered_ranges.emplace_back(clipped_t0_s, clipped_t1_s);
            first_preview_t0_s = std::isfinite(first_preview_t0_s)
                                         ? std::min(first_preview_t0_s, clipped_t0_s)
                                         : clipped_t0_s;
            ++prediction_state.orbit_plot_perf.planned_chunks_drawn;
        }

        if (full_stream_assembly)
        {
            std::vector<std::pair<double, double>> full_stream_covered_ranges;
            full_stream_covered_ranges.reserve(full_stream_assembly->chunks.size());
            draw_chunk_assembly_ranges(
                    *full_stream_assembly,
                    track_ctx.track_color_plan,
                    &covered_ranges,
                    full_stream_covered_ranges);
            covered_ranges.insert(covered_ranges.end(),
                                  full_stream_covered_ranges.begin(),
                                  full_stream_covered_ranges.end());
            const double fresh_cutoff_s =
                    std::isfinite(first_preview_t0_s) ? first_preview_t0_s : planned_window_t1_s;
            draw_cache_fallback_ranges(covered_ranges, fresh_cutoff_s);
            return;
        }

        const std::vector<std::pair<double, double>> uncovered_ranges =
                Draw::compute_uncovered_ranges(planned_window_t0_s, planned_window_t1_s, covered_ranges);
        prediction_state.orbit_plot_perf.planned_fallback_range_count = static_cast<uint32_t>(uncovered_ranges.size());
        const double drag_prefix_cutoff_s =
                track_ctx.maneuver_drag_active && std::isfinite(track_ctx.planned_draw_window.anchor_time_s)
                        ? track_ctx.planned_draw_window.anchor_time_s
                        : first_preview_t0_s;
        OrbitPredictionCache *fallback_cache = prefix_fallback_cache();
        const double fallback_prefix_cutoff_s =
                fallback_cache ? clamp_prefix_cutoff_for_cache(fallback_cache, drag_prefix_cutoff_s)
                               : std::numeric_limits<double>::quiet_NaN();
        for (const auto &[range_t0_s, range_t1_s] : uncovered_ranges)
        {
            const bool prefix_range =
                    std::isfinite(fallback_prefix_cutoff_s) &&
                    range_t1_s <= (fallback_prefix_cutoff_s + 1.0e-6);
            if (prefix_range)
            {
                if (fallback_cache)
                {
                    draw_planned_window_from_cache(*fallback_cache,
                                                   range_t0_s,
                                                   range_t1_s,
                                                   track_ctx.track_color_plan);
                }
                continue;
            }
            if (track_ctx.maneuver_drag_active || !draw_matching_cached_tail)
            {
                continue;
            }
            draw_planned_window_from_cache(planned_cache,
                                           range_t0_s,
                                           range_t1_s,
                                           preview_plan_color);
        }
    }

    void GameplayPredictionAdapter::draw_orbit_prediction_track_windows(Draw::PredictionTrackDrawContext &track_ctx)
    {
        Draw::PredictionTrackVisualPlan plan{};
        plan.track = std::move(track_ctx);
        Draw::PredictionDrawPlanner(*this).complete_visual_plan(plan);
        Draw::PredictionRenderEmitter(*this).emit(plan);
        track_ctx = std::move(plan.track);
    }
} // namespace Game
