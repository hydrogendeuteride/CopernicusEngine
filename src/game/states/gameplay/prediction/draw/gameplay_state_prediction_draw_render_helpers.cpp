#include "game/states/gameplay/prediction/draw/gameplay_state_prediction_draw_internal.h"

#include "game/orbit/orbit_plot_util.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace Game::PredictionDrawDetail
{
    namespace
    {
        WorldVec3 eval_segment_world_pos(const OrbitDrawWindowContext &ctx,
                                         const orbitsim::TrajectorySegment &segment,
                                         const double t_s)
        {
            return ctx.ref_body_world + OrbitPlotUtil::eval_segment_local_position(segment, t_s) + ctx.align_delta;
        }

        void emit_orbit_line(const OrbitDrawWindowContext &ctx,
                             const glm::vec4 &color,
                             const WorldVec3 &a_world,
                             const WorldVec3 &b_world)
        {
            if (ctx.orbit_plot)
            {
                ctx.orbit_plot->add_line(a_world, b_world, color, OrbitPlotDepth::DepthTested);
            }

            if (ctx.line_overlay_boost <= 0.0f || !ctx.orbit_plot)
            {
                return;
            }

            glm::vec4 overlay_color = color;
            overlay_color.a = std::clamp(overlay_color.a * ctx.line_overlay_boost, 0.0f, 1.0f);
            if (overlay_color.a > 0.0f)
            {
                ctx.orbit_plot->add_line(a_world, b_world, overlay_color, OrbitPlotDepth::AlwaysOnTop);
            }
        }

        void append_orbit_line_commands(std::vector<OrbitPlotSystem::LineCommand> &lines,
                                        const OrbitDrawWindowContext &ctx,
                                        const glm::vec4 &color,
                                        const WorldVec3 &a_world,
                                        const WorldVec3 &b_world)
        {
            lines.push_back(OrbitPlotSystem::LineCommand{
                    .a_world = a_world,
                    .b_world = b_world,
                    .color = color,
                    .depth = OrbitPlotDepth::DepthTested,
            });

            if (ctx.line_overlay_boost <= 0.0f)
            {
                return;
            }

            glm::vec4 overlay_color = color;
            overlay_color.a = std::clamp(overlay_color.a * ctx.line_overlay_boost, 0.0f, 1.0f);
            if (overlay_color.a > 0.0f)
            {
                lines.push_back(OrbitPlotSystem::LineCommand{
                        .a_world = a_world,
                        .b_world = b_world,
                        .color = overlay_color,
                        .depth = OrbitPlotDepth::AlwaysOnTop,
                });
            }
        }

        bool dash_budget_available(const OrbitDrawWindowContext &ctx)
        {
            return ctx.dash_chunks_remaining == nullptr || *ctx.dash_chunks_remaining > 0;
        }

        void consume_dash_budget(const OrbitDrawWindowContext &ctx)
        {
            if (ctx.dash_chunks_remaining && *ctx.dash_chunks_remaining > 0)
            {
                --(*ctx.dash_chunks_remaining);
            }
        }

        template<typename EmitLineFn>
        void emit_render_lod_segments_to(const OrbitDrawWindowContext &ctx,
                                         const OrbitPredictionDrawConfig &draw_config,
                                         OrbitPlotPerfStats &perf,
                                         const OrbitRenderCurve::RenderResult &lod,
                                         const glm::vec4 &color,
                                         const bool dashed,
                                         EmitLineFn &&emit_line)
        {
            if (lod.cap_hit)
            {
                perf.render_cap_hit_last_frame = true;
                ++perf.render_cap_hits_total;
            }
            if (lod.segments.empty())
            {
                return;
            }

            if (!dashed)
            {
                for (const OrbitRenderCurve::LineSegment &segment : lod.segments)
                {
                    emit_line(segment.a_world, segment.b_world);
                }
                return;
            }

            const double dash_on_px = draw_config.dashed_segment_on_px;
            const double dash_off_px = draw_config.dashed_segment_off_px;
            const double dash_period_px = dash_on_px + dash_off_px;
            const int max_dash_chunks_per_segment = draw_config.dash_max_chunks_per_segment;
            if (!std::isfinite(dash_on_px) || !std::isfinite(dash_off_px) ||
                !std::isfinite(dash_period_px) ||
                !(dash_on_px > 0.0) ||
                !(dash_period_px > dash_on_px) ||
                max_dash_chunks_per_segment <= 0)
            {
                for (const OrbitRenderCurve::LineSegment &segment : lod.segments)
                {
                    emit_line(segment.a_world, segment.b_world);
                }
                return;
            }

            double dash_phase_px = 0.0;
            for (const OrbitRenderCurve::LineSegment &segment : lod.segments)
            {
                if (!dash_budget_available(ctx))
                {
                    emit_line(segment.a_world, segment.b_world);
                    continue;
                }

                const double seg_m = glm::length(glm::dvec3(segment.b_world - segment.a_world));
                const double seg_dt_s = segment.t1_s - segment.t0_s;
                if (!std::isfinite(seg_m) || !(seg_m > 1.0e-9) || !std::isfinite(seg_dt_s) || !(seg_dt_s > 0.0))
                {
                    continue;
                }

                const glm::dvec3 seg_mid = glm::mix(glm::dvec3(segment.a_world), glm::dvec3(segment.b_world), 0.5);
                const double seg_mpp = meters_per_px_at_world(ctx, WorldVec3(seg_mid));
                if (!std::isfinite(seg_mpp) || !(seg_mpp > 1.0e-6))
                {
                    continue;
                }

                const double seg_px = seg_m / seg_mpp;
                if (!std::isfinite(seg_px) || !(seg_px > 1.0e-6))
                {
                    continue;
                }

                double cursor_px = 0.0;
                int dash_chunks = 0;
                while ((cursor_px + 1.0e-6) < seg_px &&
                       dash_chunks < max_dash_chunks_per_segment)
                {
                    if (!dash_budget_available(ctx))
                    {
                        const double u0 = std::clamp(cursor_px / seg_px, 0.0, 1.0);
                        if (1.0 > u0)
                        {
                            emit_line(glm::mix(segment.a_world, segment.b_world, u0), segment.b_world);
                        }
                        cursor_px = seg_px;
                        break;
                    }

                    const bool phase_on = dash_phase_px < dash_on_px;
                    double phase_remaining_px =
                            phase_on ? (dash_on_px - dash_phase_px) : (dash_period_px - dash_phase_px);
                    if (!std::isfinite(phase_remaining_px) || !(phase_remaining_px > 1.0e-6))
                    {
                        phase_remaining_px = 1.0;
                    }

                    const double step_px = std::min(phase_remaining_px, seg_px - cursor_px);
                    const double next_px = cursor_px + step_px;

                    if (phase_on)
                    {
                        const double u0 = std::clamp(cursor_px / seg_px, 0.0, 1.0);
                        const double u1 = std::clamp(next_px / seg_px, 0.0, 1.0);
                        if (u1 > u0)
                        {
                            const WorldVec3 a_world = glm::mix(segment.a_world, segment.b_world, u0);
                            const WorldVec3 b_world = glm::mix(segment.a_world, segment.b_world, u1);
                            emit_line(a_world, b_world);
                            consume_dash_budget(ctx);
                        }
                    }

                    cursor_px = next_px;
                    dash_phase_px += step_px;
                    if (dash_phase_px >= dash_period_px)
                    {
                        dash_phase_px = std::fmod(dash_phase_px, dash_period_px);
                    }
                    ++dash_chunks;
                }

                if ((cursor_px + 1.0e-6) < seg_px)
                {
                    const double remaining_px = seg_px - cursor_px;
                    dash_phase_px = std::fmod(dash_phase_px + remaining_px, dash_period_px);
                }
            }
        }

        void emit_render_lod_segments(const OrbitDrawWindowContext &ctx,
                                      const OrbitPredictionDrawConfig &draw_config,
                                      OrbitPlotPerfStats &perf,
                                      const OrbitRenderCurve::RenderResult &lod,
                                      const glm::vec4 &color,
                                      const bool dashed)
        {
            emit_render_lod_segments_to(ctx,
                                        draw_config,
                                        perf,
                                        lod,
                                        color,
                                        dashed,
                                        [&](const WorldVec3 &a_world, const WorldVec3 &b_world) {
                                            emit_orbit_line(ctx, color, a_world, b_world);
                                        });
        }

        void append_render_lod_segments(const OrbitDrawWindowContext &ctx,
                                        const OrbitPredictionDrawConfig &draw_config,
                                        OrbitPlotPerfStats &perf,
                                        const OrbitRenderCurve::RenderResult &lod,
                                        const glm::vec4 &color,
                                        const bool dashed,
                                        std::vector<OrbitPlotSystem::LineCommand> &out_lines)
        {
            emit_render_lod_segments_to(ctx,
                                        draw_config,
                                        perf,
                                        lod,
                                        color,
                                        dashed,
                                        [&](const WorldVec3 &a_world, const WorldVec3 &b_world) {
                                            append_orbit_line_commands(out_lines, ctx, color, a_world, b_world);
                                        });
        }

        uint64_t render_anchor_hash(const std::span<const double> anchor_times_s)
        {
            uint64_t seed = 0xcbf29ce484222325ULL;
            Game::prediction_hash_combine(seed, anchor_times_s.size());
            for (const double t_s : anchor_times_s)
            {
                if (std::isfinite(t_s))
                {
                    Game::prediction_hash_combine(seed, static_cast<int64_t>(std::llround(t_s * 1000.0)));
                }
            }
            return seed;
        }

        bool same_vec4(const glm::vec4 &a, const glm::vec4 &b, const float epsilon)
        {
            for (int i = 0; i < 4; ++i)
            {
                if (!std::isfinite(a[i]) || !std::isfinite(b[i]) || std::abs(a[i] - b[i]) > epsilon)
                {
                    return false;
                }
            }
            return true;
        }

        bool should_rebuild_render_line_packet_cache(const PredictionRenderLinePacketCache &cache,
                                                     const OrbitDrawWindowContext &ctx,
                                                     const glm::vec4 &color,
                                                     const double selection_error_scale,
                                                     const double t0_s,
                                                     const double t1_s,
                                                     const uint64_t generation_id,
                                                     const uint64_t display_frame_key,
                                                     const uint64_t display_frame_revision,
                                                     const uint64_t maneuver_plan_revision,
                                                     const uint64_t maneuver_plan_signature,
                                                     const uint64_t anchor_hash)
        {
            constexpr double kRenderWindowRebuildEpsilonS = 0.25;
            constexpr double kRenderOriginRebuildDistanceM = 1000.0;
            constexpr double kRenderFrustumOriginRebuildDistanceM = 1.0;
            constexpr double kRenderCameraRebuildDistanceM = 1.0;
            constexpr double kRenderMatrixRebuildEpsilon = 1.0e-6;
            constexpr double kRenderScalarRebuildEpsilon = 1.0e-6;
            constexpr float kRenderViewprojRebuildEpsilon = 1.0e-5f;
            constexpr float kRenderColorRebuildEpsilon = 1.0e-5f;

            if (!cache.valid ||
                cache.generation_id != generation_id ||
                cache.display_frame_key != display_frame_key ||
                cache.display_frame_revision != display_frame_revision ||
                cache.maneuver_plan_revision != maneuver_plan_revision ||
                cache.maneuver_plan_signature != maneuver_plan_signature ||
                cache.anchor_hash != anchor_hash ||
                cache.render_max_segments != ctx.render_max_segments)
            {
                return true;
            }

            if (!same_matrix(cache.frame_to_world, ctx.frame_to_world, kRenderMatrixRebuildEpsilon))
            {
                return true;
            }

            if (cache.render_frustum_valid != ctx.render_frustum.valid ||
                !same_matrix(cache.render_frustum_viewproj, ctx.render_frustum.viewproj, kRenderViewprojRebuildEpsilon) ||
                glm::length(glm::dvec3(cache.render_frustum_origin_world - ctx.render_frustum.origin_world)) >
                        kRenderFrustumOriginRebuildDistanceM)
            {
                return true;
            }

            const WorldVec3 line_origin_world = ctx.ref_body_world + ctx.align_delta;
            if (glm::length(glm::dvec3(cache.line_origin_world - line_origin_world)) > kRenderOriginRebuildDistanceM ||
                glm::length(cache.camera_world - ctx.camera_world) > kRenderCameraRebuildDistanceM)
            {
                return true;
            }

            if (std::abs(cache.tan_half_fov - ctx.tan_half_fov) > kRenderScalarRebuildEpsilon ||
                std::abs(cache.viewport_height_px - ctx.viewport_height_px) > kRenderScalarRebuildEpsilon ||
                std::abs(cache.render_error_px - ctx.render_error_px) > kRenderScalarRebuildEpsilon ||
                std::abs(cache.selection_error_scale - selection_error_scale) > kRenderScalarRebuildEpsilon ||
                std::abs(cache.line_overlay_boost - ctx.line_overlay_boost) > kRenderColorRebuildEpsilon ||
                !same_vec4(cache.color, color, kRenderColorRebuildEpsilon))
            {
                return true;
            }

            return !std::isfinite(cache.t0_s) || !std::isfinite(cache.t1_s) ||
                   std::abs(cache.t0_s - t0_s) > kRenderWindowRebuildEpsilonS ||
                   std::abs(cache.t1_s - t1_s) > kRenderWindowRebuildEpsilonS;
        }

        void mark_render_line_packet_cache_valid(PredictionRenderLinePacketCache &cache,
                                                 const OrbitDrawWindowContext &ctx,
                                                 const glm::vec4 &color,
                                                 const double selection_error_scale,
                                                 const double t0_s,
                                                 const double t1_s,
                                                 const uint64_t generation_id,
                                                 const uint64_t display_frame_key,
                                                 const uint64_t display_frame_revision,
                                                 const uint64_t maneuver_plan_revision,
                                                 const uint64_t maneuver_plan_signature,
                                                 const uint64_t anchor_hash,
                                                 const bool cap_hit)
        {
            cache.valid = true;
            cache.generation_id = generation_id;
            cache.display_frame_key = display_frame_key;
            cache.display_frame_revision = display_frame_revision;
            cache.maneuver_plan_revision = maneuver_plan_revision;
            cache.maneuver_plan_signature = maneuver_plan_signature;
            cache.anchor_hash = anchor_hash;
            cache.line_origin_world = ctx.ref_body_world + ctx.align_delta;
            cache.ref_body_world = ctx.ref_body_world;
            cache.align_delta_world = ctx.align_delta;
            cache.frame_to_world = ctx.frame_to_world;
            cache.render_frustum_valid = ctx.render_frustum.valid;
            cache.render_frustum_viewproj = ctx.render_frustum.viewproj;
            cache.render_frustum_origin_world = ctx.render_frustum.origin_world;
            cache.camera_world = ctx.camera_world;
            cache.tan_half_fov = ctx.tan_half_fov;
            cache.viewport_height_px = ctx.viewport_height_px;
            cache.render_error_px = ctx.render_error_px;
            cache.selection_error_scale = selection_error_scale;
            cache.t0_s = t0_s;
            cache.t1_s = t1_s;
            cache.render_max_segments = ctx.render_max_segments;
            cache.line_overlay_boost = ctx.line_overlay_boost;
            cache.color = color;
            cache.cap_hit = cap_hit;
        }

        void emit_cpu_render_lod(const OrbitDrawWindowContext &ctx,
                                 const OrbitPredictionDrawConfig &draw_config,
                                 OrbitPlotPerfStats &perf,
                                 const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                                 const double t_start_s,
                                 const double t_end_s,
                                 const glm::vec4 &color,
                                 const bool dashed)
        {
            OrbitRenderCurve::RenderSettings lod_settings{};
            lod_settings.error_px = ctx.render_error_px;
            lod_settings.max_segments = ctx.render_max_segments;

            const auto render_lod_start_tp = std::chrono::steady_clock::now();
            const OrbitRenderCurve::RenderResult lod =
                    OrbitRenderCurve::build_render_lod(traj_segments,
                                                       ctx.ref_body_world,
                                                       ctx.align_delta,
                                                       ctx.lod_camera,
                                                       lod_settings,
                                                       t_start_s,
                                                       t_end_s,
                                                       ctx.render_frustum);
            perf.render_lod_ms_last +=
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - render_lod_start_tp)
                            .count();
            emit_render_lod_segments(ctx, draw_config, perf, lod, color, dashed);
        }
    } // namespace

    void draw_orbit_window(const OrbitDrawWindowContext &ctx,
                           const OrbitPredictionDrawConfig &draw_config,
                           OrbitPlotPerfStats &perf,
                           const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                           const double t_start_s,
                           const double t_end_s,
                           const glm::vec4 &color,
                           const bool dashed)
    {
        if (!(t_end_s > t_start_s) || traj_segments.empty())
        {
            return;
        }

        const bool needs_world_basis_transform = !frame_transform_is_identity(ctx.frame_to_world);
        const std::vector<orbitsim::TrajectorySegment> transformed_segments =
                needs_world_basis_transform
                        ? transform_segments_to_world_basis(traj_segments, ctx.frame_to_world)
                        : std::vector<orbitsim::TrajectorySegment>{};
        const std::vector<orbitsim::TrajectorySegment> &segments_world_basis =
                needs_world_basis_transform ? transformed_segments : traj_segments;

        emit_cpu_render_lod(ctx, draw_config, perf, segments_world_basis, t_start_s, t_end_s, color, dashed);
    }

    void draw_adaptive_curve_window(const OrbitDrawWindowContext &ctx,
                                    const OrbitPredictionDrawConfig &draw_config,
                                    OrbitPlotPerfStats &perf,
                                    const OrbitRenderCurve &curve,
                                    const double t_start_s,
                                    const double t_end_s,
                                    const glm::vec4 &color,
                                    const bool dashed,
                                    const std::span<const double> anchor_times_s,
                                    const double selection_error_scale)
    {
        if (curve.empty() || !(t_end_s > t_start_s))
        {
            return;
        }

        OrbitRenderCurve::SelectionContext selection_ctx{};
        selection_ctx.reference_body_world = ctx.ref_body_world;
        selection_ctx.align_delta_world = ctx.align_delta;
        selection_ctx.frame_to_world = ctx.frame_to_world;
        selection_ctx.camera_world = ctx.camera_world;
        selection_ctx.tan_half_fov = ctx.tan_half_fov;
        selection_ctx.viewport_height_px = ctx.viewport_height_px;
        selection_ctx.error_frustum = ctx.render_frustum;
        selection_ctx.error_px = ctx.render_error_px;
        selection_ctx.anchor_times_s = anchor_times_s;
        if (std::isfinite(selection_error_scale) && selection_error_scale > 0.0 &&
            selection_error_scale < 1.0)
        {
            selection_ctx.error_px = std::max(0.025, ctx.render_error_px * selection_error_scale);
        }

        OrbitRenderCurve::RenderSettings render_settings{};
        render_settings.error_px = ctx.render_error_px;
        render_settings.max_segments = ctx.render_max_segments;

        const auto render_lod_start_tp = std::chrono::steady_clock::now();
        const OrbitRenderCurve::RenderResult lod =
                OrbitRenderCurve::build_render_lod(curve,
                                                   selection_ctx,
                                                   ctx.render_frustum,
                                                   render_settings,
                                                   t_start_s,
                                                   t_end_s);
        perf.render_lod_ms_last +=
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - render_lod_start_tp)
                        .count();
        if (lod.segments.empty())
        {
            return;
        }

        emit_render_lod_segments(ctx, draw_config, perf, lod, color, dashed);
    }

    void draw_cached_adaptive_curve_window(const OrbitDrawWindowContext &ctx,
                                           const OrbitPredictionDrawConfig &draw_config,
                                           OrbitPlotPerfStats &perf,
                                           const OrbitRenderCurve &curve,
                                           const double t_start_s,
                                           const double t_end_s,
                                           const glm::vec4 &color,
                                           const bool dashed,
                                           const std::span<const double> anchor_times_s,
                                           const double selection_error_scale,
                                           PredictionRenderLinePacketCache &cache,
                                           const uint64_t generation_id,
                                           const uint64_t display_frame_key,
                                           const uint64_t display_frame_revision,
                                           const uint64_t maneuver_plan_revision,
                                           const uint64_t maneuver_plan_signature)
    {
        if (dashed)
        {
            draw_adaptive_curve_window(ctx,
                                       draw_config,
                                       perf,
                                       curve,
                                       t_start_s,
                                       t_end_s,
                                       color,
                                       true,
                                       anchor_times_s,
                                       selection_error_scale);
            return;
        }

        if (curve.empty() || !(t_end_s > t_start_s) || !ctx.orbit_plot)
        {
            return;
        }

        const uint64_t anchor_hash = render_anchor_hash(anchor_times_s);
        if (!should_rebuild_render_line_packet_cache(cache,
                                                     ctx,
                                                     color,
                                                     selection_error_scale,
                                                     t_start_s,
                                                     t_end_s,
                                                     generation_id,
                                                     display_frame_key,
                                                     display_frame_revision,
                                                     maneuver_plan_revision,
                                                     maneuver_plan_signature,
                                                     anchor_hash))
        {
            if (cache.cap_hit)
            {
                perf.render_cap_hit_last_frame = true;
                ++perf.render_cap_hits_total;
            }
            if (!cache.lines.empty())
            {
                const WorldVec3 line_delta_world =
                        (ctx.ref_body_world + ctx.align_delta) - cache.line_origin_world;
                ctx.orbit_plot->add_lines_translated(
                        std::span<const OrbitPlotSystem::LineCommand>(cache.lines.data(), cache.lines.size()),
                        line_delta_world);
            }
            return;
        }

        OrbitRenderCurve::SelectionContext selection_ctx{};
        selection_ctx.reference_body_world = ctx.ref_body_world;
        selection_ctx.align_delta_world = ctx.align_delta;
        selection_ctx.frame_to_world = ctx.frame_to_world;
        selection_ctx.camera_world = ctx.camera_world;
        selection_ctx.tan_half_fov = ctx.tan_half_fov;
        selection_ctx.viewport_height_px = ctx.viewport_height_px;
        selection_ctx.error_frustum = ctx.render_frustum;
        selection_ctx.error_px = ctx.render_error_px;
        selection_ctx.anchor_times_s = anchor_times_s;
        if (std::isfinite(selection_error_scale) && selection_error_scale > 0.0 &&
            selection_error_scale < 1.0)
        {
            selection_ctx.error_px = std::max(0.025, ctx.render_error_px * selection_error_scale);
        }

        OrbitRenderCurve::RenderSettings render_settings{};
        render_settings.error_px = ctx.render_error_px;
        render_settings.max_segments = ctx.render_max_segments;

        const auto render_lod_start_tp = std::chrono::steady_clock::now();
        const OrbitRenderCurve::RenderResult lod =
                OrbitRenderCurve::build_render_lod(curve,
                                                   selection_ctx,
                                                   ctx.render_frustum,
                                                   render_settings,
                                                   t_start_s,
                                                   t_end_s);
        perf.render_lod_ms_last +=
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - render_lod_start_tp)
                        .count();

        cache.lines.clear();
        cache.lines.reserve(lod.segments.size() * ((ctx.line_overlay_boost > 0.0f) ? 2u : 1u));
        append_render_lod_segments(ctx, draw_config, perf, lod, color, false, cache.lines);
        mark_render_line_packet_cache_valid(cache,
                                            ctx,
                                            color,
                                            selection_error_scale,
                                            t_start_s,
                                            t_end_s,
                                            generation_id,
                                            display_frame_key,
                                            display_frame_revision,
                                            maneuver_plan_revision,
                                            maneuver_plan_signature,
                                            anchor_hash,
                                            lod.cap_hit);

        if (!cache.lines.empty())
        {
            ctx.orbit_plot->add_lines(std::span<const OrbitPlotSystem::LineCommand>(cache.lines.data(),
                                                                                    cache.lines.size()));
        }
    }

    namespace
    {
        double snap_time_past_straddling_chunk(const PredictionChunkAssembly &planned_assembly,
                                               const double t_s)
        {
            for (const OrbitChunk &chunk : planned_assembly.chunks)
            {
                if (!chunk.valid ||
                    chunk.frame_segments.empty() ||
                    !std::isfinite(chunk.t0_s) ||
                    !std::isfinite(chunk.t1_s) ||
                    t_s < chunk.t0_s ||
                    t_s > chunk.t1_s)
                {
                    continue;
                }
                return snap_time_past_straddling_segment(chunk.frame_segments, t_s);
            }
            return t_s;
        }

        PickWindow build_planned_window_from_policy(const PredictionChunkAssembly &planned_assembly,
                                                    const OrbitPredictionDrawConfig &draw_config,
                                                    const double anchor_time_s,
                                                    const bool anchor_is_future,
                                                    const double window_span_s,
                                                    const double window_start_time_s = std::numeric_limits<double>::quiet_NaN(),
                                                    const double window_end_time_s = std::numeric_limits<double>::quiet_NaN())
        {
            PickWindow planned_window{};
            double t0p = 0.0;
            double t1p = 0.0;
            if (!planned_assembly.time_span(t0p, t1p) || !std::isfinite(anchor_time_s))
            {
                return planned_window;
            }

            if (!(t1p > t0p))
            {
                return planned_window;
            }

            double t_plan_start = std::clamp(anchor_time_s, t0p, t1p);
            if (anchor_is_future)
            {
                const double snapped_t_plan_start =
                        std::clamp(snap_time_past_straddling_chunk(planned_assembly, t_plan_start), t0p, t1p);
                if (snapped_t_plan_start > (t_plan_start + draw_config.node_time_tolerance_s))
                {
                    t_plan_start = snapped_t_plan_start;
                }
            }

            if (!(window_span_s > 0.0))
            {
                return planned_window;
            }

            if (std::isfinite(window_start_time_s))
            {
                t_plan_start = std::clamp(window_start_time_s, t0p, t1p);
            }

            double t_plan_end = std::min(t_plan_start + window_span_s, t1p);
            if (std::isfinite(window_end_time_s))
            {
                t_plan_end = std::clamp(window_end_time_s, t0p, t1p);
            }
            if (!(t_plan_end > t_plan_start))
            {
                return planned_window;
            }

            planned_window.valid = true;
            planned_window.t0_s = t_plan_start;
            planned_window.t1_s = t_plan_end;
            planned_window.anchor_time_s = anchor_time_s;
            return planned_window;
        }
    } // namespace

    PickWindow build_planned_draw_window(const PredictionChunkAssembly &planned_assembly,
                                         const OrbitPredictionDrawConfig &draw_config,
                                         const PredictionWindowPolicyResult &policy)
    {
        if (!policy.valid)
        {
            return {};
        }
        return build_planned_window_from_policy(planned_assembly,
                                                draw_config,
                                                policy.visual_anchor_time_s,
                                                policy.visual_anchor_is_future,
                                                policy.visual_window_s,
                                                policy.visual_window_start_time_s,
                                                policy.visual_window_end_time_s);
    }

    PickWindow build_planned_pick_window(const PredictionChunkAssembly &planned_assembly,
                                         const OrbitPredictionDrawConfig &draw_config,
                                         const PredictionWindowPolicyResult &policy)
    {
        if (!policy.valid)
        {
            return {};
        }
        return build_planned_window_from_policy(planned_assembly,
                                                draw_config,
                                                policy.pick_anchor_time_s,
                                                policy.pick_anchor_is_future,
                                                policy.pick_window_s,
                                                policy.pick_window_start_time_s,
                                                policy.pick_window_end_time_s);
    }

    std::size_t build_pick_segment_cache(const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                                         const WorldVec3 &ref_body_world,
                                         const glm::dmat3 &frame_to_world,
                                         const WorldVec3 &align_delta,
                                         const OrbitRenderCurve::FrustumContext &pick_frustum,
                                         const OrbitRenderCurve::PickSettings &pick_settings,
                                         const double t0_s,
                                         const double t1_s,
                                         const std::span<const double> anchor_times_s,
                                         const bool segments_are_world_basis,
                                         std::vector<PickingSystem::LinePickSegmentData> &out_segments,
                                         bool &out_cap_hit,
                                         OrbitPlotPerfStats &perf)
    {
        const bool needs_world_basis_transform =
                !segments_are_world_basis && !frame_transform_is_identity(frame_to_world);
        const std::vector<orbitsim::TrajectorySegment> transformed_segments =
                needs_world_basis_transform
                        ? transform_segments_to_world_basis(traj_segments, frame_to_world)
                        : std::vector<orbitsim::TrajectorySegment>{};
        const std::vector<orbitsim::TrajectorySegment> &segments_world_basis =
                needs_world_basis_transform ? transformed_segments : traj_segments;
        out_segments.clear();
        out_cap_hit = false;
        if (segments_world_basis.empty())
        {
            return 0;
        }

        const auto pick_start_tp = std::chrono::steady_clock::now();
        const OrbitRenderCurve::PickResult lod =
                OrbitRenderCurve::build_pick_lod(segments_world_basis,
                                                 ref_body_world,
                                                 align_delta,
                                                 pick_frustum,
                                                 pick_settings,
                                                 t0_s,
                                                 t1_s,
                                                 anchor_times_s);
        perf.pick_lod_ms_last +=
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - pick_start_tp).count();
        perf.pick_segments_before_cull += static_cast<uint32_t>(lod.segments_before_cull);
        perf.pick_segments += static_cast<uint32_t>(lod.segments_after_cull);

        out_segments.reserve(lod.segments.size());
        for (const OrbitRenderCurve::LineSegment &segment : lod.segments)
        {
            out_segments.push_back(PickingSystem::LinePickSegmentData{
                    .a_world = segment.a_world,
                    .b_world = segment.b_world,
                    .a_time_s = segment.t0_s,
                    .b_time_s = segment.t1_s,
            });
        }
        const std::size_t emitted = out_segments.size();
        out_cap_hit = lod.cap_hit;
        perf.pick_cap_hit_last_frame = perf.pick_cap_hit_last_frame || out_cap_hit;
        if (out_cap_hit)
        {
            ++perf.pick_cap_hits_total;
        }
        return emitted;
    }

    bool frame_spec_uses_direct_world_polyline(const orbitsim::TrajectoryFrameSpec &spec)
    {
        (void) spec;
        return false;
    }

    void draw_polyline_window(const OrbitDrawWindowContext &ctx,
                              const OrbitPredictionDrawConfig &draw_config,
                              const std::vector<orbitsim::TrajectorySample> &traj,
                              const double t_start_s,
                              const double t_end_s,
                              const glm::vec4 &color,
                              const bool dashed)
    {
        if (!(t_end_s > t_start_s) || traj.size() < 2)
        {
            return;
        }

        const double dash_on_px = draw_config.dashed_segment_on_px;
        const double dash_off_px = draw_config.dashed_segment_off_px;
        const double dash_period_px = dash_on_px + dash_off_px;
        const int max_dash_chunks_per_segment = draw_config.dash_max_chunks_per_segment;
        if (dashed &&
            (!std::isfinite(dash_on_px) ||
             !std::isfinite(dash_off_px) ||
             !std::isfinite(dash_period_px) ||
             !(dash_on_px > 0.0) ||
             !(dash_period_px > dash_on_px) ||
             max_dash_chunks_per_segment <= 0))
        {
            draw_polyline_window(ctx, draw_config, traj, t_start_s, t_end_s, color, false);
            return;
        }
        double dash_phase_px = 0.0;

        for (std::size_t i = 1; i < traj.size(); ++i)
        {
            const double seg_t0_s = traj[i - 1].t_s;
            const double seg_t1_s = traj[i].t_s;
            const double clip_t0_s = std::max(seg_t0_s, t_start_s);
            const double clip_t1_s = std::min(seg_t1_s, t_end_s);
            if (!(clip_t1_s > clip_t0_s))
            {
                continue;
            }

            const WorldVec3 a_world =
                    sample_polyline_world(ctx.ref_body_world, ctx.frame_to_world, traj, i - 1, i, clip_t0_s) + ctx.align_delta;
            const WorldVec3 b_world =
                    sample_polyline_world(ctx.ref_body_world, ctx.frame_to_world, traj, i - 1, i, clip_t1_s) + ctx.align_delta;

            if (!dashed)
            {
                emit_orbit_line(ctx, color, a_world, b_world);
                continue;
            }

            if (!dash_budget_available(ctx))
            {
                emit_orbit_line(ctx, color, a_world, b_world);
                continue;
            }

            const double seg_m = glm::length(glm::dvec3(b_world - a_world));
            const glm::dvec3 seg_mid = glm::mix(glm::dvec3(a_world), glm::dvec3(b_world), 0.5);
            const double seg_mpp = meters_per_px_at_world(ctx, WorldVec3(seg_mid));
            const double seg_px = seg_m / seg_mpp;
            if (!std::isfinite(seg_px) || !(seg_px > 1.0e-6) || !std::isfinite(seg_mpp) || !(seg_mpp > 1.0e-6))
            {
                continue;
            }

            double cursor_px = 0.0;
            int dash_chunks = 0;
            while ((cursor_px + 1.0e-6) < seg_px &&
                   dash_chunks < max_dash_chunks_per_segment)
            {
                if (!dash_budget_available(ctx))
                {
                    const double u0 = std::clamp(cursor_px / seg_px, 0.0, 1.0);
                    if (1.0 > u0)
                    {
                        emit_orbit_line(ctx, color, glm::mix(a_world, b_world, u0), b_world);
                    }
                    cursor_px = seg_px;
                    break;
                }

                const bool phase_on = dash_phase_px < dash_on_px;
                double phase_remaining_px = phase_on ? (dash_on_px - dash_phase_px) : (dash_period_px - dash_phase_px);
                if (!std::isfinite(phase_remaining_px) || !(phase_remaining_px > 1.0e-6))
                {
                    phase_remaining_px = 1.0;
                }

                const double step_px = std::min(phase_remaining_px, seg_px - cursor_px);
                const double next_px = cursor_px + step_px;
                if (phase_on)
                {
                    const double u0 = std::clamp(cursor_px / seg_px, 0.0, 1.0);
                    const double u1 = std::clamp(next_px / seg_px, 0.0, 1.0);
                    emit_orbit_line(ctx, color, glm::mix(a_world, b_world, u0), glm::mix(a_world, b_world, u1));
                    consume_dash_budget(ctx);
                }

                cursor_px = next_px;
                dash_phase_px += step_px;
                if (dash_phase_px >= dash_period_px)
                {
                    dash_phase_px = std::fmod(dash_phase_px, dash_period_px);
                }
                ++dash_chunks;
            }
        }
    }

    std::size_t emit_polyline_pick_segments(const OrbitDrawWindowContext &ctx,
                                            PickingSystem *picking,
                                            const uint32_t pick_group,
                                            const std::vector<orbitsim::TrajectorySample> &traj,
                                            const double t0_s,
                                            const double t1_s,
                                            const std::size_t max_segments,
                                            OrbitPlotPerfStats &perf)
    {
        if (!picking || pick_group == kInvalidPickGroup || traj.size() < 2)
        {
            return 0;
        }

        std::size_t emitted = 0;
        for (std::size_t i = 1; i < traj.size() && emitted < max_segments; ++i)
        {
            const double seg_t0_s = traj[i - 1].t_s;
            const double seg_t1_s = traj[i].t_s;
            const double clip_t0_s = std::max(seg_t0_s, t0_s);
            const double clip_t1_s = std::min(seg_t1_s, t1_s);
            if (!(clip_t1_s > clip_t0_s))
            {
                continue;
            }

            const WorldVec3 a_world =
                    sample_polyline_world(ctx.ref_body_world, ctx.frame_to_world, traj, i - 1, i, clip_t0_s) + ctx.align_delta;
            const WorldVec3 b_world =
                    sample_polyline_world(ctx.ref_body_world, ctx.frame_to_world, traj, i - 1, i, clip_t1_s) + ctx.align_delta;
            picking->add_line_pick_segment(pick_group, a_world, b_world, clip_t0_s, clip_t1_s);
            ++emitted;
        }

        perf.pick_segments_before_cull += static_cast<uint32_t>(emitted);
        perf.pick_segments += static_cast<uint32_t>(emitted);
        return emitted;
    }

    void emit_velocity_ray(GameAPI::Engine *api,
                           const WorldVec3 &ship_pos_world,
                           const glm::dvec3 &ship_vel_world,
                           const float ttl_s,
                           const glm::vec4 &color)
    {
        const double speed_mps = glm::length(ship_vel_world);
        double len_m = 40.0;
        if (std::isfinite(speed_mps) && speed_mps > 1.0)
        {
            len_m = std::clamp(speed_mps * 0.002, 10.0, 250.0);
        }

        api->debug_draw_ray(glm::dvec3(ship_pos_world), ship_vel_world, len_m, color, ttl_s, true);
    }
} // namespace Game::PredictionDrawDetail
