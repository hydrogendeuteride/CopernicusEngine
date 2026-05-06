#include "game/states/gameplay/prediction_nbody/draw/gameplay_state_prediction_draw_internal.h"

#include <algorithm>
#include <cmath>

namespace Game::PredictionDrawDetail
{
    namespace
    {
        constexpr double kPickWindowRebuildEpsilonS = 0.25;
        constexpr double kPickAlignRebuildDistanceM = 25.0;
        constexpr double kPickReferenceRebuildDistanceM = 1.0;
        constexpr double kPickFrustumOriginRebuildDistanceM = 1.0;
        constexpr double kPickCameraRebuildDistanceM = 1.0;
        constexpr double kPickMatrixRebuildEpsilon = 1.0e-6;
        constexpr double kPickScalarRebuildEpsilon = 1.0e-6;
        constexpr float kPickViewprojRebuildEpsilon = 1.0e-5f;
        constexpr double kPickAnchorWindowEpsilonS = 1.0e-6;

        bool time_in_pick_window(const PickWindow &window, const double t_s)
        {
            return window.valid &&
                   std::isfinite(t_s) &&
                   std::isfinite(window.t0_s) &&
                   std::isfinite(window.t1_s) &&
                   window.t1_s >= window.t0_s &&
                   t_s >= (window.t0_s - kPickAnchorWindowEpsilonS) &&
                   t_s <= (window.t1_s + kPickAnchorWindowEpsilonS);
        }

        void normalize_time_cache(PredictionTimeAnchorCache &cache)
        {
            std::sort(cache.times_s.begin(), cache.times_s.end());
            cache.times_s.erase(std::unique(cache.times_s.begin(), cache.times_s.end()), cache.times_s.end());
        }

        void rebuild_maneuver_node_time_cache(PredictionTimeAnchorCache &cache,
                                              const std::vector<ManeuverNode> &nodes,
                                              const uint64_t maneuver_revision)
        {
            if (cache.generation_id == 0 &&
                cache.revision == maneuver_revision &&
                cache.source_count == nodes.size())
            {
                return;
            }

            cache.generation_id = 0;
            cache.revision = maneuver_revision;
            cache.source_count = nodes.size();
            cache.times_s.clear();
            cache.times_s.reserve(nodes.size());
            for (const ManeuverNode &node : nodes)
            {
                if (std::isfinite(node.time_s))
                {
                    cache.times_s.push_back(node.time_s);
                }
            }
            normalize_time_cache(cache);
        }

        void rebuild_preview_time_cache(PredictionTimeAnchorCache &cache,
                                        const OrbitPredictionCache &prediction_cache)
        {
            const std::vector<OrbitPredictionManeuverNodePreview> &previews =
                    prediction_cache.solver.planned.maneuver_previews;
            const uint64_t generation_id = prediction_cache.identity.generation_id;
            const uint64_t revision = prediction_cache.identity.maneuver_plan_revision;
            if (cache.generation_id == generation_id &&
                cache.revision == revision &&
                cache.source_count == previews.size())
            {
                return;
            }

            cache.generation_id = generation_id;
            cache.revision = revision;
            cache.source_count = previews.size();
            cache.times_s.clear();
            cache.times_s.reserve(previews.size());
            for (const OrbitPredictionManeuverNodePreview &preview : previews)
            {
                if (preview.valid && std::isfinite(preview.t_s))
                {
                    cache.times_s.push_back(preview.t_s);
                }
            }
            normalize_time_cache(cache);
        }

        void append_interior_anchor_times(const std::vector<double> &sorted_times,
                                          const double window_t0_s,
                                          const double window_t1_s,
                                          std::vector<double> &anchors)
        {
            if (sorted_times.empty() ||
                !std::isfinite(window_t0_s) ||
                !std::isfinite(window_t1_s) ||
                !(window_t1_s > window_t0_s))
            {
                return;
            }

            const double lo_s = window_t0_s + kPickAnchorWindowEpsilonS;
            const double hi_s = window_t1_s - kPickAnchorWindowEpsilonS;
            if (!(hi_s > lo_s))
            {
                return;
            }

            const auto first = std::upper_bound(sorted_times.begin(), sorted_times.end(), lo_s);
            const auto last = std::lower_bound(first, sorted_times.end(), hi_s);
            anchors.insert(anchors.end(), first, last);
        }

        void append_interior_anchor_time(const double t_s,
                                         const double window_t0_s,
                                         const double window_t1_s,
                                         std::vector<double> &anchors)
        {
            if (!std::isfinite(t_s))
            {
                return;
            }
            if (t_s <= (window_t0_s + kPickAnchorWindowEpsilonS) ||
                t_s >= (window_t1_s - kPickAnchorWindowEpsilonS))
            {
                return;
            }
            anchors.push_back(t_s);
        }
    }

    const std::vector<orbitsim::TrajectorySegment> &base_segments_world_basis(PredictionTrackDrawContext &track_ctx)
    {
        if (track_ctx.identity_frame_transform)
        {
            return *track_ctx.traj_base_segments;
        }

        if (track_ctx.traj_base_segments_world_basis.empty() && track_ctx.traj_base_segments &&
            !track_ctx.traj_base_segments->empty())
        {
            track_ctx.traj_base_segments_world_basis =
                    transform_segments_to_world_basis(*track_ctx.traj_base_segments, track_ctx.frame_to_world);
        }
        return track_ctx.traj_base_segments_world_basis;
    }

    std::vector<double> collect_pick_anchor_times(const std::vector<ManeuverNode> &nodes,
                                                  const PickWindow &base_pick_window,
                                                  const PickWindow &planned_pick_window,
                                                  const double now_s)
    {
        std::vector<double> anchor_times;
        anchor_times.reserve(nodes.size() + 2u);

        const auto push_anchor_if_in_window = [&](const double t_s) {
            if (time_in_pick_window(base_pick_window, t_s) || time_in_pick_window(planned_pick_window, t_s))
            {
                anchor_times.push_back(t_s);
            }
        };

        push_anchor_if_in_window(now_s);
        for (const ManeuverNode &node : nodes)
        {
            push_anchor_if_in_window(node.time_s);
        }
        push_anchor_if_in_window(planned_pick_window.anchor_time_s);

        std::sort(anchor_times.begin(), anchor_times.end());
        anchor_times.erase(std::unique(anchor_times.begin(), anchor_times.end()), anchor_times.end());
        return anchor_times;
    }

    std::vector<double> collect_planned_curve_anchor_times(PredictionTimeAnchorCache &maneuver_node_cache,
                                                           PredictionTimeAnchorCache &preview_time_cache,
                                                           const std::vector<ManeuverNode> &nodes,
                                                           const uint64_t maneuver_revision,
                                                           const OrbitPredictionCache &cache,
                                                           const double preview_anchor_time_s,
                                                           const bool preview_anchor_valid,
                                                           const double window_t0_s,
                                                           const double window_t1_s)
    {
        std::vector<double> anchors;
        if (!std::isfinite(window_t0_s) || !std::isfinite(window_t1_s) || !(window_t1_s > window_t0_s))
        {
            return anchors;
        }

        rebuild_maneuver_node_time_cache(maneuver_node_cache, nodes, maneuver_revision);
        rebuild_preview_time_cache(preview_time_cache, cache);

        anchors.reserve(8u);
        anchors.push_back(window_t0_s);
        append_interior_anchor_times(preview_time_cache.times_s, window_t0_s, window_t1_s, anchors);
        append_interior_anchor_times(maneuver_node_cache.times_s, window_t0_s, window_t1_s, anchors);
        if (preview_anchor_valid)
        {
            append_interior_anchor_time(preview_anchor_time_s, window_t0_s, window_t1_s, anchors);
        }
        anchors.push_back(window_t1_s);

        std::sort(anchors.begin(), anchors.end());
        anchors.erase(std::unique(anchors.begin(), anchors.end()), anchors.end());
        return anchors;
    }

    bool same_matrix(const glm::dmat3 &a, const glm::dmat3 &b, const double epsilon)
    {
        for (int col = 0; col < 3; ++col)
        {
            for (int row = 0; row < 3; ++row)
            {
                if (!std::isfinite(a[col][row]) || !std::isfinite(b[col][row]) ||
                    std::abs(a[col][row] - b[col][row]) > epsilon)
                {
                    return false;
                }
            }
        }
        return true;
    }

    bool same_matrix(const glm::mat4 &a, const glm::mat4 &b, const float epsilon)
    {
        for (int col = 0; col < 4; ++col)
        {
            for (int row = 0; row < 4; ++row)
            {
                if (!std::isfinite(a[col][row]) || !std::isfinite(b[col][row]) ||
                    std::abs(a[col][row] - b[col][row]) > epsilon)
                {
                    return false;
                }
            }
        }
        return true;
    }

    std::vector<std::pair<double, double>> compute_uncovered_ranges(
            const double t_start_s,
            const double t_end_s,
            std::vector<std::pair<double, double>> covered_ranges)
    {
        std::vector<std::pair<double, double>> uncovered_ranges;
        if (!(t_end_s > t_start_s))
        {
            return uncovered_ranges;
        }

        if (covered_ranges.empty())
        {
            uncovered_ranges.emplace_back(t_start_s, t_end_s);
            return uncovered_ranges;
        }

        std::sort(covered_ranges.begin(),
                  covered_ranges.end(),
                  [](const auto &a, const auto &b) {
                      if (a.first == b.first)
                      {
                          return a.second < b.second;
                      }
                      return a.first < b.first;
                  });

        constexpr double kTimeEpsilonS = 1.0e-6;
        double cursor_t_s = t_start_s;
        for (const auto &[covered_t0_s, covered_t1_s] : covered_ranges)
        {
            if (!(covered_t1_s > covered_t0_s))
            {
                continue;
            }

            const double clamped_t0_s = std::max(covered_t0_s, t_start_s);
            const double clamped_t1_s = std::min(covered_t1_s, t_end_s);
            if (!(clamped_t1_s > clamped_t0_s))
            {
                continue;
            }

            if (clamped_t0_s > (cursor_t_s + kTimeEpsilonS))
            {
                uncovered_ranges.emplace_back(cursor_t_s, clamped_t0_s);
            }
            cursor_t_s = std::max(cursor_t_s, clamped_t1_s);
        }

        if (t_end_s > (cursor_t_s + kTimeEpsilonS))
        {
            uncovered_ranges.emplace_back(cursor_t_s, t_end_s);
        }

        return uncovered_ranges;
    }

    bool should_rebuild_pick_cache(const PredictionLinePickCache &cache,
                                   const uint64_t generation_id,
                                   const uint64_t display_frame_key,
                                   const uint64_t display_frame_revision,
                                   const WorldVec3 &ref_body_world,
                                   const glm::dmat3 &frame_to_world,
                                   const WorldVec3 &align_delta,
                                   const glm::dvec3 &camera_world,
                                   const double tan_half_fov,
                                   const double viewport_height_px,
                                   const double render_error_px,
                                   const OrbitRenderCurve::FrustumContext &pick_frustum,
                                   const double pick_frustum_margin_ratio,
                                   const double t0_s,
                                   const double t1_s,
                                   const std::size_t max_segments,
                                   const bool use_adaptive_curve,
                                   const bool planned)
    {
        const bool valid = planned ? cache.planned_valid : cache.base_valid;
        const double cached_t0_s = planned ? cache.planned_t0_s : cache.base_t0_s;
        const double cached_t1_s = planned ? cache.planned_t1_s : cache.base_t1_s;
        const std::size_t cached_max_segments = planned ? cache.planned_max_segments : cache.base_max_segments;
        const bool cached_use_adaptive_curve = planned ? cache.planned_use_adaptive_curve : cache.base_use_adaptive_curve;
        if (!valid || cache.generation_id != generation_id ||
            cache.display_frame_key != display_frame_key ||
            cache.display_frame_revision != display_frame_revision ||
            cached_max_segments != max_segments)
        {
            return true;
        }

        if (cached_use_adaptive_curve != use_adaptive_curve)
        {
            return true;
        }

        if (!same_matrix(cache.frame_to_world, frame_to_world, kPickMatrixRebuildEpsilon))
        {
            return true;
        }

        if (cache.pick_frustum_valid != pick_frustum.valid ||
            !same_matrix(cache.pick_frustum_viewproj, pick_frustum.viewproj, kPickViewprojRebuildEpsilon) ||
            glm::length(glm::dvec3(cache.pick_frustum_origin_world - pick_frustum.origin_world)) >
                    kPickFrustumOriginRebuildDistanceM ||
            std::abs(cache.pick_frustum_margin_ratio - pick_frustum_margin_ratio) > kPickMatrixRebuildEpsilon)
        {
            return true;
        }

        if (glm::length(glm::dvec3(cache.ref_body_world - ref_body_world)) > kPickReferenceRebuildDistanceM ||
            glm::length(glm::dvec3(cache.align_delta_world - align_delta)) > kPickAlignRebuildDistanceM)
        {
            return true;
        }

        if (use_adaptive_curve &&
            (glm::length(cache.camera_world - camera_world) > kPickCameraRebuildDistanceM ||
             std::abs(cache.tan_half_fov - tan_half_fov) > kPickScalarRebuildEpsilon ||
             std::abs(cache.viewport_height_px - viewport_height_px) > kPickScalarRebuildEpsilon ||
             std::abs(cache.render_error_px - render_error_px) > kPickScalarRebuildEpsilon))
        {
            return true;
        }

        return !std::isfinite(cached_t0_s) || !std::isfinite(cached_t1_s) ||
               std::abs(cached_t0_s - t0_s) > kPickWindowRebuildEpsilonS ||
               std::abs(cached_t1_s - t1_s) > kPickWindowRebuildEpsilonS;
    }

    void mark_pick_cache_valid(PredictionLinePickCache &cache,
                               const uint64_t generation_id,
                               const uint64_t display_frame_key,
                               const uint64_t display_frame_revision,
                               const WorldVec3 &ref_body_world,
                               const glm::dmat3 &frame_to_world,
                               const WorldVec3 &align_delta,
                               const glm::dvec3 &camera_world,
                               const double tan_half_fov,
                               const double viewport_height_px,
                               const double render_error_px,
                               const OrbitRenderCurve::FrustumContext &pick_frustum,
                               const double pick_frustum_margin_ratio,
                               const double t0_s,
                               const double t1_s,
                               const std::size_t max_segments,
                               const bool use_adaptive_curve,
                               const bool planned)
    {
        cache.generation_id = generation_id;
        cache.display_frame_key = display_frame_key;
        cache.display_frame_revision = display_frame_revision;
        cache.ref_body_world = ref_body_world;
        cache.frame_to_world = frame_to_world;
        cache.align_delta_world = align_delta;
        cache.camera_world = camera_world;
        cache.tan_half_fov = tan_half_fov;
        cache.viewport_height_px = viewport_height_px;
        cache.render_error_px = render_error_px;
        cache.pick_frustum_valid = pick_frustum.valid;
        cache.pick_frustum_viewproj = pick_frustum.viewproj;
        cache.pick_frustum_origin_world = pick_frustum.origin_world;
        cache.pick_frustum_margin_ratio = pick_frustum_margin_ratio;
        if (planned)
        {
            cache.planned_valid = true;
            cache.planned_t0_s = t0_s;
            cache.planned_t1_s = t1_s;
            cache.planned_max_segments = max_segments;
            cache.planned_use_adaptive_curve = use_adaptive_curve;
        }
        else
        {
            cache.base_valid = true;
            cache.base_t0_s = t0_s;
            cache.base_t1_s = t1_s;
            cache.base_max_segments = max_segments;
            cache.base_use_adaptive_curve = use_adaptive_curve;
        }
    }

    bool same_pick_time_window(const double cached_t0_s,
                               const double cached_t1_s,
                               const double t0_s,
                               const double t1_s)
    {
        return std::isfinite(cached_t0_s) &&
               std::isfinite(cached_t1_s) &&
               std::isfinite(t0_s) &&
               std::isfinite(t1_s) &&
               std::abs(cached_t0_s - t0_s) <= kPickWindowRebuildEpsilonS &&
               std::abs(cached_t1_s - t1_s) <= kPickWindowRebuildEpsilonS;
    }

} // namespace Game::PredictionDrawDetail
