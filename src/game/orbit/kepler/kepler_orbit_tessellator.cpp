#include "game/orbit/kepler/kepler_orbit_tessellator.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

namespace Game
{
    namespace
    {
        bool finite_vec3(const orbitsim::Vec3 &v)
        {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        }

        bool finite_state(const orbitsim::State &state)
        {
            return finite_vec3(state.position_m) &&
                   finite_vec3(state.velocity_mps) &&
                   finite_vec3(state.spin.axis) &&
                   std::isfinite(state.spin.angle_rad) &&
                   std::isfinite(state.spin.rate_rad_per_s);
        }

        bool same_sample_time(const double a, const double b)
        {
            return std::isfinite(a) && std::isfinite(b) && std::abs(a - b) <= 1.0e-9;
        }

        bool resolve_provider_state(const KeplerBodyStateProvider &provider,
                                    const orbitsim::BodyId body_id,
                                    const double t_s,
                                    const orbitsim::State &fallback,
                                    orbitsim::State &out_state)
        {
            if (provider.state_at && provider.state_at(body_id, t_s, out_state) && finite_state(out_state))
            {
                return true;
            }

            out_state = fallback;
            return finite_state(out_state);
        }

        std::size_t allowed_samples_for_arc(const KeplerOrbitTessellationOptions &options,
                                            const std::size_t remaining_total)
        {
            if (remaining_total == 0u || options.max_vertices_per_arc == 0u)
            {
                return 0u;
            }
            return std::min(options.max_vertices_per_arc, remaining_total);
        }

        double sample_dt_for_arc(const orbitsim::KeplerArc &arc,
                                 const KeplerOrbitTessellationOptions &options,
                                 const std::size_t allowed_samples)
        {
            const double duration_s = std::abs(arc.t1_s - arc.t0_s);
            const double configured_dt =
                    (std::isfinite(options.max_time_step_s) && options.max_time_step_s > 0.0)
                            ? options.max_time_step_s
                            : 60.0;
            const double min_dt =
                    (std::isfinite(options.min_time_step_s) && options.min_time_step_s > 0.0)
                            ? options.min_time_step_s
                            : 1.0;
            double sample_dt_s = std::max(min_dt, configured_dt);
            if (allowed_samples > 1u && duration_s > 0.0 && std::isfinite(duration_s))
            {
                const double required_dt_s = duration_s / static_cast<double>(allowed_samples - 1u);
                if (std::isfinite(required_dt_s) && required_dt_s > sample_dt_s)
                {
                    sample_dt_s = required_dt_s;
                }
            }
            return sample_dt_s;
        }

        double positive_or_default(const double value, const double fallback)
        {
            return (std::isfinite(value) && value > 0.0) ? value : fallback;
        }

        double point_segment_distance_m(const WorldVec3 &p,
                                        const WorldVec3 &a,
                                        const WorldVec3 &b)
        {
            const double ab_x = b.x - a.x;
            const double ab_y = b.y - a.y;
            const double ab_z = b.z - a.z;
            const double ap_x = p.x - a.x;
            const double ap_y = p.y - a.y;
            const double ap_z = p.z - a.z;
            const double ab_len2 = ab_x * ab_x + ab_y * ab_y + ab_z * ab_z;
            if (!(ab_len2 > 0.0) || !std::isfinite(ab_len2))
            {
                return std::sqrt(ap_x * ap_x + ap_y * ap_y + ap_z * ap_z);
            }

            const double t = std::clamp((ap_x * ab_x + ap_y * ab_y + ap_z * ab_z) / ab_len2,
                                        0.0,
                                        1.0);
            const double closest_x = a.x + t * ab_x;
            const double closest_y = a.y + t * ab_y;
            const double closest_z = a.z + t * ab_z;
            const double dx = p.x - closest_x;
            const double dy = p.y - closest_y;
            const double dz = p.z - closest_z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        struct LineSampleEvaluation
        {
            bool ok{false};
            KeplerOrbitStatus status{KeplerOrbitStatus::PropagationFailed};
            orbitsim::KeplerStatus kepler_status{orbitsim::KeplerStatus::Ok};
            KeplerOrbitLineVertex vertex{};
        };

        struct AdaptiveInterval
        {
            std::size_t a_index{0u};
            std::size_t b_index{0u};
            double priority{0.0};
            KeplerOrbitLineVertex midpoint{};
        };

        struct AdaptiveIntervalPriority
        {
            bool operator()(const AdaptiveInterval &a, const AdaptiveInterval &b) const
            {
                return a.priority < b.priority;
            }
        };

        struct AdaptiveArcLineBuild
        {
            bool ok{false};
            KeplerOrbitStatus status{KeplerOrbitStatus::InvalidInput};
            orbitsim::KeplerStatus first_kepler_failure{orbitsim::KeplerStatus::Ok};
            std::size_t requested_samples{0u};
            std::size_t accepted_samples{0u};
            bool budget_hit{false};
            std::vector<KeplerOrbitLineVertex> vertices{};
        };

        LineSampleEvaluation evaluate_line_sample(const KeplerOrbitTessellationRequest &request,
                                                  const KeplerOrbitArc &game_arc,
                                                  const double t_s)
        {
            LineSampleEvaluation out{};
            const orbitsim::KeplerArcSample sample =
                    orbitsim::sample_kepler_arc_state(game_arc.arc,
                                                      t_s,
                                                      request.options.propagation);
            if (!sample.ok())
            {
                out.status = KeplerOrbitStatus::PropagationFailed;
                out.kepler_status = sample.diagnostics.status;
                return out;
            }

            orbitsim::State primary_state{};
            if (!resolve_provider_state(request.body_state_provider,
                                        game_arc.arc.primary_body_id,
                                        sample.t_s,
                                        game_arc.primary_state_inertial_at_t0,
                                        primary_state))
            {
                out.status = KeplerOrbitStatus::InvalidPrimary;
                return out;
            }

            orbitsim::State reference_state{};
            if (request.world_frame.world_reference_body_id != orbitsim::kInvalidBodyId)
            {
                if (!resolve_provider_state(request.body_state_provider,
                                            request.world_frame.world_reference_body_id,
                                            sample.t_s,
                                            request.world_frame.world_reference_state_inertial,
                                            reference_state))
                {
                    out.status = KeplerOrbitStatus::InvalidPrimary;
                    return out;
                }
            }
            else
            {
                reference_state = request.world_frame.world_reference_state_inertial;
            }

            const orbitsim::Vec3 inertial_position_m =
                    primary_state.position_m + sample.state_relative.position_m;
            const WorldVec3 world_position =
                    request.world_frame.world_reference_body_world +
                    WorldVec3(inertial_position_m - reference_state.position_m);

            out.ok = true;
            out.status = KeplerOrbitStatus::Ok;
            out.vertex = KeplerOrbitLineVertex{
                    .position_world = world_position,
                    .t_s = sample.t_s,
                    .primary_body_id = game_arc.arc.primary_body_id,
                    .flags = 0u,
            };
            return out;
        }

        AdaptiveArcLineBuild build_adaptive_arc_lines(const KeplerOrbitTessellationRequest &request,
                                                      const KeplerOrbitArc &game_arc,
                                                      const std::size_t arc_index,
                                                      const std::size_t arc_count,
                                                      const std::size_t allowed_samples)
        {
            AdaptiveArcLineBuild out{};
            if (allowed_samples == 0u)
            {
                out.status = KeplerOrbitStatus::SampleBudgetExceeded;
                out.budget_hit = true;
                return out;
            }

            std::vector<KeplerOrbitLineVertex> samples{};
            samples.reserve(std::min<std::size_t>(allowed_samples, 256u));

            const auto evaluate = [&](const double t_s) {
                ++out.requested_samples;
                LineSampleEvaluation eval = evaluate_line_sample(request, game_arc, t_s);
                if (eval.ok)
                {
                    ++out.accepted_samples;
                }
                return eval;
            };

            const LineSampleEvaluation start_eval = evaluate(game_arc.arc.t0_s);
            if (!start_eval.ok)
            {
                out.status = start_eval.status;
                out.first_kepler_failure = start_eval.kepler_status;
                return out;
            }
            samples.push_back(start_eval.vertex);

            const double duration_s = std::abs(game_arc.arc.t1_s - game_arc.arc.t0_s);
            if (duration_s > 0.0 && std::isfinite(duration_s))
            {
                if (allowed_samples > 1u)
                {
                    const LineSampleEvaluation end_eval = evaluate(game_arc.arc.t1_s);
                    if (!end_eval.ok)
                    {
                        out.status = end_eval.status;
                        out.first_kepler_failure = end_eval.kepler_status;
                        return out;
                    }
                    samples.push_back(end_eval.vertex);
                }
                else
                {
                    out.budget_hit = true;
                }
            }

            const double max_interval_s = sample_dt_for_arc(game_arc.arc, request.options, allowed_samples);
            const double min_interval_s = positive_or_default(request.options.min_time_step_s, 1.0);
            const double max_chord_error_m = request.options.max_chord_error_m;
            const bool chord_error_enabled =
                    std::isfinite(max_chord_error_m) && max_chord_error_m > 0.0;

            std::priority_queue<AdaptiveInterval,
                                std::vector<AdaptiveInterval>,
                                AdaptiveIntervalPriority>
                    intervals{};
            bool failed = false;
            LineSampleEvaluation failed_eval{};

            const auto try_push_interval = [&](const std::size_t a_index,
                                               const std::size_t b_index) {
                if (failed || a_index >= samples.size() || b_index >= samples.size())
                {
                    return;
                }
                if (samples.size() >= allowed_samples)
                {
                    out.budget_hit = true;
                    return;
                }

                const KeplerOrbitLineVertex &a = samples[a_index];
                const KeplerOrbitLineVertex &b = samples[b_index];
                const double dt_s = std::abs(b.t_s - a.t_s);
                if (!(dt_s > min_interval_s) || !std::isfinite(dt_s))
                {
                    return;
                }

                const bool split_for_time =
                        std::isfinite(max_interval_s) &&
                        max_interval_s > 0.0 &&
                        dt_s > max_interval_s * (1.0 + 1.0e-9);
                if (!split_for_time && !chord_error_enabled)
                {
                    return;
                }

                const double mid_t_s = 0.5 * (a.t_s + b.t_s);
                const double min_t_s = std::min(a.t_s, b.t_s);
                const double max_t_s = std::max(a.t_s, b.t_s);
                if (!(mid_t_s > min_t_s) || !(mid_t_s < max_t_s) || !std::isfinite(mid_t_s))
                {
                    return;
                }

                const LineSampleEvaluation mid_eval = evaluate(mid_t_s);
                if (!mid_eval.ok)
                {
                    failed = true;
                    failed_eval = mid_eval;
                    return;
                }

                double priority = 0.0;
                if (split_for_time)
                {
                    priority = std::max(priority, dt_s / max_interval_s);
                }

                bool split_for_error = false;
                if (chord_error_enabled)
                {
                    const double error_m = point_segment_distance_m(mid_eval.vertex.position_world,
                                                                    a.position_world,
                                                                    b.position_world);
                    split_for_error = std::isfinite(error_m) && error_m > max_chord_error_m;
                    if (split_for_error)
                    {
                        priority = std::max(priority, error_m / max_chord_error_m);
                    }
                }

                if (!split_for_time && !split_for_error)
                {
                    return;
                }

                if (!(priority > 0.0) || !std::isfinite(priority))
                {
                    priority = 1.0;
                }

                intervals.push(AdaptiveInterval{
                        .a_index = a_index,
                        .b_index = b_index,
                        .priority = priority,
                        .midpoint = mid_eval.vertex,
                });
            };

            if (samples.size() >= 2u)
            {
                try_push_interval(0u, 1u);
            }
            if (failed)
            {
                out.status = failed_eval.status;
                out.first_kepler_failure = failed_eval.kepler_status;
                return out;
            }

            while (!intervals.empty())
            {
                if (samples.size() >= allowed_samples)
                {
                    out.budget_hit = true;
                    break;
                }

                const AdaptiveInterval interval = intervals.top();
                intervals.pop();
                const std::size_t mid_index = samples.size();
                samples.push_back(interval.midpoint);
                try_push_interval(interval.a_index, mid_index);
                try_push_interval(mid_index, interval.b_index);
                if (failed)
                {
                    out.status = failed_eval.status;
                    out.first_kepler_failure = failed_eval.kepler_status;
                    return out;
                }
            }

            std::sort(samples.begin(),
                      samples.end(),
                      [](const KeplerOrbitLineVertex &a, const KeplerOrbitLineVertex &b) {
                          return a.t_s < b.t_s;
                      });

            for (const KeplerOrbitLineVertex &sample : samples)
            {
                const bool is_start = same_sample_time(sample.t_s, game_arc.arc.t0_s);
                const bool is_end = same_sample_time(sample.t_s, game_arc.arc.t1_s);
                if ((!request.options.include_start && is_start) ||
                    (!request.options.include_end && is_end))
                {
                    continue;
                }
                if (!out.vertices.empty() && same_sample_time(out.vertices.back().t_s, sample.t_s))
                {
                    continue;
                }
                out.vertices.push_back(sample);
            }

            for (std::size_t i = 0; i < out.vertices.size(); ++i)
            {
                uint32_t flags = 0u;
                if (arc_index == 0u && i == 0u)
                {
                    flags |= static_cast<uint32_t>(KeplerOrbitLineVertexFlags::OrbitStart);
                }
                if (arc_index + 1u == arc_count && i + 1u == out.vertices.size())
                {
                    flags |= static_cast<uint32_t>(KeplerOrbitLineVertexFlags::OrbitEnd);
                }
                if (i == 0u)
                {
                    flags |= static_cast<uint32_t>(KeplerOrbitLineVertexFlags::ArcStart);
                }
                if (i + 1u == out.vertices.size())
                {
                    flags |= static_cast<uint32_t>(KeplerOrbitLineVertexFlags::ArcEnd);
                }
                out.vertices[i].flags = flags;
            }

            out.ok = true;
            out.status = KeplerOrbitStatus::Ok;
            return out;
        }

        void append_vertex(KeplerOrbitLineSet &out,
                           const KeplerOrbitLineVertex &vertex)
        {
            if (!out.vertices.empty() && same_sample_time(out.vertices.back().t_s, vertex.t_s))
            {
                out.vertices.back().flags |= vertex.flags;
                return;
            }
            out.vertices.push_back(vertex);
        }

        bool finish_partial_line_set(KeplerOrbitLineSet &out,
                                     const KeplerOrbitStatus status,
                                     const std::size_t failed_arc_index,
                                     const orbitsim::KeplerStatus first_kepler_failure = orbitsim::KeplerStatus::Ok)
        {
            out.diagnostics.status = status;
            out.diagnostics.failed_arc_index = failed_arc_index;
            if (first_kepler_failure != orbitsim::KeplerStatus::Ok)
            {
                out.diagnostics.first_kepler_failure = first_kepler_failure;
            }
            if (out.vertices.size() >= 2u)
            {
                out.valid = true;
                return true;
            }
            return false;
        }
    } // namespace

    KeplerOrbitLineSet build_kepler_orbit_lines(const KeplerOrbitTessellationRequest &request)
    {
        KeplerOrbitLineSet out{};
        out.diagnostics.requested_arcs = request.arcs.size();
        if (request.arcs.empty() || request.options.max_vertices_total == 0u)
        {
            out.diagnostics.status = KeplerOrbitStatus::InvalidInput;
            return out;
        }

        for (std::size_t arc_index = 0; arc_index < request.arcs.size(); ++arc_index)
        {
            if (out.vertices.size() >= request.options.max_vertices_total)
            {
                out.diagnostics.budget_hit = true;
                break;
            }

            const KeplerOrbitArc &game_arc = request.arcs[arc_index];
            if (!orbitsim::kepler_arc_valid(game_arc.arc))
            {
                out.diagnostics.status = KeplerOrbitStatus::InvalidArc;
                out.diagnostics.failed_arc_index = arc_index;
                return out;
            }

            const std::size_t remaining_total = request.options.max_vertices_total - out.vertices.size();
            const std::size_t allowed_samples = allowed_samples_for_arc(request.options, remaining_total);
            if (allowed_samples == 0u)
            {
                out.diagnostics.budget_hit = true;
                break;
            }

            const AdaptiveArcLineBuild arc_lines =
                    build_adaptive_arc_lines(request,
                                             game_arc,
                                             arc_index,
                                             request.arcs.size(),
                                             allowed_samples);
            out.diagnostics.requested_samples += arc_lines.requested_samples;
            out.diagnostics.accepted_samples += arc_lines.accepted_samples;
            out.diagnostics.budget_hit = out.diagnostics.budget_hit || arc_lines.budget_hit;
            if (!arc_lines.ok)
            {
                (void) finish_partial_line_set(out,
                                               arc_lines.status,
                                               arc_index,
                                               arc_lines.first_kepler_failure);
                return out;
            }

            for (const KeplerOrbitLineVertex &vertex : arc_lines.vertices)
            {
                append_vertex(out, vertex);
            }

            ++out.diagnostics.sampled_arcs;
        }

        if (out.vertices.size() < 2u)
        {
            out.diagnostics.status = KeplerOrbitStatus::NoSamples;
            return out;
        }

        out.valid = true;
        if (out.diagnostics.status == KeplerOrbitStatus::InvalidInput)
        {
            out.diagnostics.status = KeplerOrbitStatus::Ok;
        }
        return out;
    }

} // namespace Game
