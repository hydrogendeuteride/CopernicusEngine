#include "game/orbit/kepler/kepler_arc_line_builder.h"

#include "orbitsim/math.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

namespace Game
{
    // Choose a sample interval that respects the vertex budget.
    double sample_dt_for_arc(const orbitsim::KeplerArc &arc,
                             const KeplerArcLineOptions &options,
                             const std::size_t allowed_samples)
    {
        const double duration_s = std::abs(arc.t1_s - arc.t0_s);
        const double configured_dt = kepler_positive_or_default(options.max_time_step_s, 60.0);
        const double min_dt = kepler_positive_or_default(options.min_time_step_s, 1.0);
        double sample_dt_s = std::max(min_dt, configured_dt);
        if (allowed_samples > 1u && duration_s > 0.0)
        {
            const double required_dt_s = duration_s / static_cast<double>(allowed_samples - 1u);
            if (required_dt_s > sample_dt_s)
            {
                sample_dt_s = required_dt_s;
            }
        }
        return sample_dt_s;
    }

    // Detect a closed orbit ending near its start point.
    bool arc_end_returns_to_start(const orbitsim::KeplerArc &arc)
    {
        const orbitsim::OrbitalElements elements =
                orbitsim::orbital_elements_from_relative_state(arc.mu_m3_s2,
                                                               arc.state0_relative.position_m,
                                                               arc.state0_relative.velocity_mps);
        if (!(elements.eccentricity < 1.0))
        {
            return false;
        }

        const orbitsim::OrbitScalars scalars =
                orbitsim::orbit_scalars_from_elements(arc.mu_m3_s2, elements);
        const double period_s = scalars.valid ? scalars.period_s : 0.0;
        const double duration_s = std::abs(arc.t1_s - arc.t0_s);
        if (!(period_s > 0.0) || !(duration_s > 0.0))
        {
            return false;
        }

        const double period_count = duration_s / period_s;
        const double nearest_period_count = std::round(period_count);
        return nearest_period_count >= 1.0 &&
               std::abs(period_count - nearest_period_count) <= 1.0e-3;
    }

    // Chord error estimate for adaptive line refinement.
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
        if (!(ab_len2 > 0.0))
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

    // One sampled point plus propagation failure details.
    struct LineSampleEvaluation
    {
        bool ok{false};
        orbitsim::KeplerStatus kepler_status{orbitsim::KeplerStatus::Ok};
        KeplerArcLineVertex vertex{};
    };

    // Candidate segment that may need another midpoint.
    struct AdaptiveInterval
    {
        std::size_t a_index{0u};
        std::size_t b_index{0u};
        double priority{0.0};
        KeplerArcLineVertex midpoint{};
    };

    struct AdaptiveIntervalPriority
    {
        bool operator()(const AdaptiveInterval &a, const AdaptiveInterval &b) const
        {
            return a.priority < b.priority;
        }
    };

    // Per-arc line build accumulator.
    struct AdaptiveArcLineBuild
    {
        bool ok{false};
        KeplerOrbitStatus status{KeplerOrbitStatus::InvalidInput};
        orbitsim::KeplerStatus first_kepler_failure{orbitsim::KeplerStatus::Ok};
        std::size_t requested_samples{0u};
        std::size_t accepted_samples{0u};
        bool budget_hit{false};
        std::vector<KeplerArcLineVertex> vertices{};
    };

    // Sampling limits for one arc.
    struct AdaptiveLineConfig
    {
        double duration_s{0.0};
        double max_interval_s{60.0};
        double min_interval_s{1.0};
        double max_chord_error_m{0.0};
        bool chord_error_enabled{false};
    };

    using AdaptiveIntervalQueue =
            std::priority_queue<AdaptiveInterval,
                                std::vector<AdaptiveInterval>,
                                AdaptiveIntervalPriority>;

    bool finite_state(const orbitsim::State &state)
    {
        return kepler_finite_vec3(state.position_m) && kepler_finite_vec3(state.velocity_mps);
    }

    bool solve_hyperbolic_anomaly(const double eccentricity,
                                  const double mean_anomaly_rad,
                                  double &out_hyperbolic_anomaly_rad)
    {
        if (!(eccentricity > 1.0) || !std::isfinite(eccentricity) || !std::isfinite(mean_anomaly_rad))
        {
            return false;
        }

        double h = std::asinh(mean_anomaly_rad / eccentricity);
        if (!std::isfinite(h))
        {
            return false;
        }

        constexpr int kMaxIterations = 32;
        constexpr double kTolerance = 1.0e-12;
        for (int i = 0; i < kMaxIterations; ++i)
        {
            const double sinh_h = std::sinh(h);
            const double cosh_h = std::cosh(h);
            const double f = eccentricity * sinh_h - h - mean_anomaly_rad;
            const double df = eccentricity * cosh_h - 1.0;
            if (!std::isfinite(f) || !(std::abs(df) > 0.0) || !std::isfinite(df))
            {
                return false;
            }

            const double step = f / df;
            h -= step;
            if (!std::isfinite(h))
            {
                return false;
            }
            if (std::abs(step) <= kTolerance * std::max(1.0, std::abs(h)))
            {
                out_hyperbolic_anomaly_rad = h;
                return true;
            }
        }

        out_hyperbolic_anomaly_rad = h;
        return std::isfinite(h);
    }

    bool sample_hyperbolic_arc_state_from_elements(const orbitsim::KeplerArc &arc,
                                                   const double t_s,
                                                   orbitsim::State &out_state)
    {
        if (!orbitsim::kepler_arc_valid(arc) || !std::isfinite(t_s))
        {
            return false;
        }

        const orbitsim::OrbitalElements elements =
                orbitsim::orbital_elements_from_relative_state(arc.mu_m3_s2,
                                                               arc.state0_relative.position_m,
                                                               arc.state0_relative.velocity_mps);
        if (!(elements.eccentricity > 1.0) ||
            !(elements.semi_major_axis_m < 0.0) ||
            !std::isfinite(elements.semi_major_axis_m) ||
            !std::isfinite(elements.mean_anomaly_rad))
        {
            return false;
        }

        const double abs_a = -elements.semi_major_axis_m;
        const double mean_motion_radps = std::sqrt(arc.mu_m3_s2 / (abs_a * abs_a * abs_a));
        if (!(mean_motion_radps > 0.0) || !std::isfinite(mean_motion_radps))
        {
            return false;
        }

        const double mean_anomaly_rad =
                elements.mean_anomaly_rad + mean_motion_radps * (t_s - arc.t0_s);
        double hyperbolic_anomaly_rad = 0.0;
        if (!solve_hyperbolic_anomaly(elements.eccentricity,
                                      mean_anomaly_rad,
                                      hyperbolic_anomaly_rad))
        {
            return false;
        }

        const double tanh_half_h = std::tanh(0.5 * hyperbolic_anomaly_rad);
        const double true_anomaly_rad =
                2.0 * std::atan(std::sqrt((elements.eccentricity + 1.0) /
                                          (elements.eccentricity - 1.0)) *
                                 tanh_half_h);
        if (!std::isfinite(true_anomaly_rad))
        {
            return false;
        }

        orbitsim::OrbitalElements propagated = elements;
        propagated.true_anomaly_rad = true_anomaly_rad;
        out_state = orbitsim::relative_state_from_orbital_elements(arc.mu_m3_s2, propagated);
        out_state.spin = arc.state0_relative.spin;
        return finite_state(out_state);
    }

    void record_propagation_failure(AdaptiveArcLineBuild &build,
                                    const orbitsim::KeplerStatus status)
    {
        build.status = KeplerOrbitStatus::PropagationFailed;
        if (build.first_kepler_failure == orbitsim::KeplerStatus::Ok)
        {
            build.first_kepler_failure = status;
        }
    }

    // Propagate one time sample and convert it into world space.
    LineSampleEvaluation evaluate_line_sample(const KeplerArcLineBuildRequest &request,
                                              const KeplerOrbitArc &game_arc,
                                              const double t_s,
                                              AdaptiveArcLineBuild &build)
    {
        ++build.requested_samples;

        LineSampleEvaluation out{};
        const orbitsim::KeplerArcSample sample =
                orbitsim::sample_kepler_arc_state(game_arc.arc,
                                                  t_s,
                                                  request.options.propagation);
        orbitsim::State sample_state = sample.state_relative;
        if (!sample.ok() &&
            !sample_hyperbolic_arc_state_from_elements(game_arc.arc, t_s, sample_state))
        {
            out.kepler_status = sample.diagnostics.status;
            return out;
        }

        // Prefer predicted body states when a provider is available.
        orbitsim::State primary_state = game_arc.primary_state_inertial_at_t0;
        if (request.body_state_provider.state_at)
        {
            orbitsim::State provider_state{};
            if (request.body_state_provider.state_at(game_arc.arc.primary_body_id,
                                                     sample.t_s,
                                                     provider_state))
            {
                primary_state = provider_state;
            }
        }

        // Keep the line in the current world reference frame.
        orbitsim::State reference_state = request.world_frame.world_reference_state_inertial;
        if (request.world_frame.world_reference_body_id != orbitsim::kInvalidBodyId)
        {
            if (request.body_state_provider.state_at)
            {
                orbitsim::State provider_state{};
                if (request.body_state_provider.state_at(request.world_frame.world_reference_body_id,
                                                         sample.t_s,
                                                         provider_state))
                {
                    reference_state = provider_state;
                }
            }
        }

        const orbitsim::Vec3 inertial_position_m =
                primary_state.position_m + sample_state.position_m;
        const WorldVec3 world_position =
                request.world_frame.world_reference_body_world +
                WorldVec3(inertial_position_m - reference_state.position_m);

        out.ok = true;
        out.vertex = KeplerArcLineVertex{
                .position_world = world_position,
                .t_s = sample.t_s,
                .primary_body_id = game_arc.arc.primary_body_id,
                .flags = 0u,
        };
        ++build.accepted_samples;
        return out;
    }

    // Keep open-orbit lines drawable when an extreme far endpoint exceeds the solver range.
    bool append_last_valid_sample_before_failed_end(const KeplerArcLineBuildRequest &request,
                                                    const KeplerOrbitArc &game_arc,
                                                    const AdaptiveLineConfig &config,
                                                    const KeplerArcLineVertex &known_good,
                                                    const double failed_t_s,
                                                    AdaptiveArcLineBuild &out,
                                                    std::vector<KeplerArcLineVertex> &samples)
    {
        constexpr int kMaxClipIterations = 48;
        double good_t_s = known_good.t_s;
        double bad_t_s = failed_t_s;
        LineSampleEvaluation best_eval{};
        best_eval.ok = true;
        best_eval.vertex = known_good;

        for (int i = 0; i < kMaxClipIterations; ++i)
        {
            const double mid_t_s = 0.5 * (good_t_s + bad_t_s);
            if (!std::isfinite(mid_t_s) ||
                kepler_same_sample_time(mid_t_s, good_t_s) ||
                kepler_same_sample_time(mid_t_s, bad_t_s))
            {
                break;
            }

            const LineSampleEvaluation mid_eval =
                    evaluate_line_sample(request, game_arc, mid_t_s, out);
            if (mid_eval.ok)
            {
                best_eval = mid_eval;
                good_t_s = mid_t_s;
            }
            else
            {
                record_propagation_failure(out, mid_eval.kepler_status);
                bad_t_s = mid_t_s;
            }
        }

        if (kepler_same_sample_time(best_eval.vertex.t_s, known_good.t_s) ||
            std::abs(best_eval.vertex.t_s - known_good.t_s) <= config.min_interval_s)
        {
            return false;
        }

        samples.push_back(best_eval.vertex);
        return true;
    }

    // Seed endpoints before adaptive midpoint insertion.
    bool seed_arc_line_samples(const KeplerArcLineBuildRequest &request,
                               const KeplerOrbitArc &game_arc,
                               const AdaptiveLineConfig &config,
                               const std::size_t arc_index,
                               const std::size_t allowed_samples,
                               AdaptiveArcLineBuild &out,
                               std::vector<KeplerArcLineVertex> &samples)
    {
        double end_t_s = game_arc.arc.t1_s;
        if (arc_index == 0u &&
            request.options.include_start &&
            allowed_samples > 2u &&
            config.duration_s > 0.0 &&
            arc_end_returns_to_start(game_arc.arc))
        {
            constexpr double kPreferredCrossDtS = 0.25;
            const double cross_dt_s =
                    std::min({kPreferredCrossDtS, config.max_interval_s, config.duration_s * 0.25});
            LineSampleEvaluation eval =
                    evaluate_line_sample(request, game_arc, game_arc.arc.t0_s - cross_dt_s, out);
            if (!eval.ok)
            {
                record_propagation_failure(out, eval.kepler_status);
                return false;
            }
            samples.push_back(eval.vertex);

            eval = evaluate_line_sample(request, game_arc, game_arc.arc.t0_s + cross_dt_s, out);
            if (!eval.ok)
            {
                record_propagation_failure(out, eval.kepler_status);
                return false;
            }
            samples.push_back(eval.vertex);
            end_t_s = game_arc.arc.t1_s - cross_dt_s;
        }
        else
        {
            LineSampleEvaluation eval =
                    evaluate_line_sample(request, game_arc, game_arc.arc.t0_s, out);
            if (!eval.ok)
            {
                record_propagation_failure(out, eval.kepler_status);
                return false;
            }
            samples.push_back(eval.vertex);
        }

        if (!(config.duration_s > 0.0))
        {
            return true;
        }
        if (samples.size() >= allowed_samples)
        {
            out.budget_hit = true;
            return true;
        }

        const LineSampleEvaluation end_eval =
                evaluate_line_sample(request, game_arc, end_t_s, out);
        if (!end_eval.ok)
        {
            record_propagation_failure(out, end_eval.kepler_status);
            return append_last_valid_sample_before_failed_end(request,
                                                              game_arc,
                                                              config,
                                                              samples.back(),
                                                              end_t_s,
                                                              out,
                                                              samples);
        }
        samples.push_back(end_eval.vertex);
        return true;
    }

    // Queue intervals that violate time or chord-error limits.
    bool queue_adaptive_interval(const KeplerArcLineBuildRequest &request,
                                 const KeplerOrbitArc &game_arc,
                                 const AdaptiveLineConfig &config,
                                 const std::size_t allowed_samples,
                                 AdaptiveArcLineBuild &out,
                                 std::vector<KeplerArcLineVertex> &samples,
                                 AdaptiveIntervalQueue &intervals,
                                 const std::size_t a_index,
                                 const std::size_t b_index)
    {
        if (samples.size() >= allowed_samples)
        {
            out.budget_hit = true;
            return true;
        }

        const KeplerArcLineVertex &a = samples[a_index];
        const KeplerArcLineVertex &b = samples[b_index];
        const double dt_s = std::abs(b.t_s - a.t_s);
        if (!(dt_s > config.min_interval_s))
        {
            return true;
        }

        const bool split_for_time =
                dt_s > config.max_interval_s * (1.0 + 1.0e-9);
        if (!split_for_time && !config.chord_error_enabled)
        {
            return true;
        }

        const double mid_t_s = 0.5 * (a.t_s + b.t_s);
        const LineSampleEvaluation mid_eval =
                evaluate_line_sample(request, game_arc, mid_t_s, out);
        if (!mid_eval.ok)
        {
            record_propagation_failure(out, mid_eval.kepler_status);
            return true;
        }

        double priority = split_for_time ? dt_s / config.max_interval_s : 0.0;
        bool split_for_error = false;
        if (config.chord_error_enabled)
        {
            const double error_m = point_segment_distance_m(mid_eval.vertex.position_world,
                                                            a.position_world,
                                                            b.position_world);
            split_for_error = error_m > config.max_chord_error_m;
            if (split_for_error)
            {
                priority = std::max(priority, error_m / config.max_chord_error_m);
            }
        }

        if (!split_for_time && !split_for_error)
        {
            return true;
        }

        if (!(priority > 0.0))
        {
            priority = 1.0;
        }

        intervals.push(AdaptiveInterval{
                .a_index = a_index,
                .b_index = b_index,
                .priority = priority,
                .midpoint = mid_eval.vertex,
        });
        return true;
    }

    // Split queued intervals until quality targets or budget limits are reached.
    bool refine_arc_line_samples(const KeplerArcLineBuildRequest &request,
                                 const KeplerOrbitArc &game_arc,
                                 const AdaptiveLineConfig &config,
                                 const std::size_t allowed_samples,
                                 AdaptiveArcLineBuild &out,
                                 std::vector<KeplerArcLineVertex> &samples)
    {
        AdaptiveIntervalQueue intervals{};
        for (std::size_t i = 1u; i < samples.size(); ++i)
        {
            if (!queue_adaptive_interval(request,
                                         game_arc,
                                         config,
                                         allowed_samples,
                                         out,
                                         samples,
                                         intervals,
                                         i - 1u,
                                         i))
            {
                return false;
            }
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

            if (!queue_adaptive_interval(request,
                                         game_arc,
                                         config,
                                         allowed_samples,
                                         out,
                                         samples,
                                         intervals,
                                         interval.a_index,
                                         mid_index) ||
                !queue_adaptive_interval(request,
                                         game_arc,
                                         config,
                                         allowed_samples,
                                         out,
                                         samples,
                                         intervals,
                                         mid_index,
                                         interval.b_index))
            {
                return false;
            }
        }
        return true;
    }

    // Mark orbit and per-arc boundary vertices.
    uint32_t arc_line_vertex_flags(const std::size_t vertex_index,
                                   const std::size_t vertex_count,
                                   const std::size_t arc_index,
                                   const std::size_t arc_count)
    {
        uint32_t flags = 0u;
        if (arc_index == 0u && vertex_index == 0u)
        {
            flags |= static_cast<uint32_t>(KeplerArcLineVertexFlags::OrbitStart);
        }
        if (arc_index + 1u == arc_count && vertex_index + 1u == vertex_count)
        {
            flags |= static_cast<uint32_t>(KeplerArcLineVertexFlags::OrbitEnd);
        }
        if (vertex_index == 0u)
        {
            flags |= static_cast<uint32_t>(KeplerArcLineVertexFlags::ArcStart);
        }
        if (vertex_index + 1u == vertex_count)
        {
            flags |= static_cast<uint32_t>(KeplerArcLineVertexFlags::ArcEnd);
        }
        return flags;
    }

    // Sort samples, drop disabled endpoints, and apply boundary flags.
    void emit_arc_line_vertices(const KeplerArcLineBuildRequest &request,
                                const KeplerOrbitArc &game_arc,
                                const std::size_t arc_index,
                                const std::size_t arc_count,
                                std::vector<KeplerArcLineVertex> &samples,
                                AdaptiveArcLineBuild &out)
    {
        std::sort(samples.begin(),
                  samples.end(),
                  [](const KeplerArcLineVertex &a, const KeplerArcLineVertex &b) {
                      return a.t_s < b.t_s;
                  });

        for (const KeplerArcLineVertex &sample : samples)
        {
            const bool is_start = kepler_same_sample_time(sample.t_s, game_arc.arc.t0_s);
            const bool is_end = kepler_same_sample_time(sample.t_s, game_arc.arc.t1_s);
            if ((!request.options.include_start && is_start) ||
                (!request.options.include_end && is_end))
            {
                continue;
            }
            if (!out.vertices.empty() &&
                kepler_same_sample_time(out.vertices.back().t_s, sample.t_s))
            {
                continue;
            }
            out.vertices.push_back(sample);
        }

        const std::size_t vertex_count = out.vertices.size();
        for (std::size_t i = 0; i < vertex_count; ++i)
        {
            out.vertices[i].flags = arc_line_vertex_flags(i, vertex_count, arc_index, arc_count);
        }
    }

    // Build vertices for a single arc.
    AdaptiveArcLineBuild build_adaptive_arc_lines(const KeplerArcLineBuildRequest &request,
                                                  const KeplerOrbitArc &game_arc,
                                                  const std::size_t arc_index,
                                                  const std::size_t arc_count,
                                                  const std::size_t allowed_samples)
    {
        AdaptiveArcLineBuild out{};
        std::vector<KeplerArcLineVertex> samples{};
        samples.reserve(std::min<std::size_t>(allowed_samples, 256u));
        const double max_chord_error_m = request.options.max_chord_error_m;
        const AdaptiveLineConfig config{
                .duration_s = std::abs(game_arc.arc.t1_s - game_arc.arc.t0_s),
                .max_interval_s = sample_dt_for_arc(game_arc.arc, request.options, allowed_samples),
                .min_interval_s = kepler_positive_or_default(request.options.min_time_step_s, 1.0),
                .max_chord_error_m = max_chord_error_m,
                .chord_error_enabled = max_chord_error_m > 0.0,
        };
        if (!seed_arc_line_samples(request,
                                   game_arc,
                                   config,
                                   arc_index,
                                   allowed_samples,
                                   out,
                                   samples) ||
            !refine_arc_line_samples(request,
                                     game_arc,
                                     config,
                                     allowed_samples,
                                     out,
                                     samples))
        {
            return out;
        }

        emit_arc_line_vertices(request, game_arc, arc_index, arc_count, samples, out);
        out.ok = true;
        if (out.status == KeplerOrbitStatus::InvalidInput)
        {
            out.status = KeplerOrbitStatus::Ok;
        }
        return out;
    }

    // Build one continuous line set from all requested arcs.
    KeplerArcLineSet build_kepler_arc_lines(const KeplerArcLineBuildRequest &request)
    {
        KeplerArcLineSet out{};
        out.diagnostics.requested_arcs = request.arcs.size();
        if (request.arcs.empty() || request.options.max_vertices_total == 0u)
        {
            out.diagnostics.status = KeplerOrbitStatus::InvalidInput;
            return out;
        }
        out.diagnostics.status = KeplerOrbitStatus::Ok;

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
            const std::size_t allowed_samples =
                    std::min(request.options.max_vertices_per_arc, remaining_total);
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
            if (arc_lines.status != KeplerOrbitStatus::Ok &&
                out.diagnostics.status == KeplerOrbitStatus::Ok)
            {
                out.diagnostics.status = arc_lines.status;
                out.diagnostics.failed_arc_index = arc_index;
                if (arc_lines.first_kepler_failure != orbitsim::KeplerStatus::Ok)
                {
                    out.diagnostics.first_kepler_failure = arc_lines.first_kepler_failure;
                }
            }
            if (!arc_lines.ok)
            {
                out.diagnostics.status = arc_lines.status;
                out.diagnostics.failed_arc_index = arc_index;
                if (arc_lines.first_kepler_failure != orbitsim::KeplerStatus::Ok)
                {
                    out.diagnostics.first_kepler_failure = arc_lines.first_kepler_failure;
                }
                out.valid = out.vertices.size() >= 2u;
                return out;
            }

            for (const KeplerArcLineVertex &vertex : arc_lines.vertices)
            {
                if (!out.vertices.empty() && kepler_same_sample_time(out.vertices.back().t_s, vertex.t_s))
                {
                    out.vertices.back().flags |= vertex.flags;
                    continue;
                }
                out.vertices.push_back(vertex);
            }

            ++out.diagnostics.sampled_arcs;
        }

        if (out.vertices.size() < 2u)
        {
            out.diagnostics.status = KeplerOrbitStatus::NoSamples;
            return out;
        }

        out.valid = true;
        return out;
    }

} // namespace Game
