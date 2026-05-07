#include "game/orbit/kepler/kepler_orbit_tessellator.h"

#include <algorithm>
#include <cmath>

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

            orbitsim::KeplerTrajectoryOptions sample_options{};
            sample_options.sample_dt_s = sample_dt_for_arc(game_arc.arc, request.options, allowed_samples);
            sample_options.max_samples = allowed_samples;
            sample_options.include_start = request.options.include_start;
            sample_options.include_end = request.options.include_end;
            sample_options.propagation = request.options.propagation;

            orbitsim::KeplerTrajectoryDiagnostics sample_diagnostics{};
            const std::vector<orbitsim::KeplerArcSample> samples =
                    orbitsim::build_kepler_arc_samples(game_arc.arc, sample_options, &sample_diagnostics);
            out.diagnostics.requested_samples += sample_diagnostics.requested_samples;
            out.diagnostics.accepted_samples += sample_diagnostics.accepted_samples;
            if (sample_diagnostics.first_failure != orbitsim::KeplerStatus::Ok)
            {
                out.diagnostics.status = KeplerOrbitStatus::PropagationFailed;
                out.diagnostics.failed_arc_index = arc_index;
                out.diagnostics.first_kepler_failure = sample_diagnostics.first_failure;
                return out;
            }

            if (samples.size() == allowed_samples &&
                request.options.include_end &&
                !samples.empty() &&
                !same_sample_time(samples.back().t_s, game_arc.arc.t1_s))
            {
                out.diagnostics.budget_hit = true;
            }

            for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index)
            {
                const orbitsim::KeplerArcSample &sample = samples[sample_index];
                if (!sample.ok())
                {
                    out.diagnostics.status = KeplerOrbitStatus::PropagationFailed;
                    out.diagnostics.failed_arc_index = arc_index;
                    out.diagnostics.first_kepler_failure = sample.diagnostics.status;
                    return out;
                }

                orbitsim::State primary_state{};
                if (!resolve_provider_state(request.body_state_provider,
                                            game_arc.arc.primary_body_id,
                                            sample.t_s,
                                            game_arc.primary_state_inertial_at_t0,
                                            primary_state))
                {
                    out.diagnostics.status = KeplerOrbitStatus::InvalidPrimary;
                    out.diagnostics.failed_arc_index = arc_index;
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
                        out.diagnostics.status = KeplerOrbitStatus::InvalidPrimary;
                        out.diagnostics.failed_arc_index = arc_index;
                        return out;
                    }
                }
                else
                {
                    reference_state = request.world_frame.world_reference_state_inertial;
                }

                uint32_t flags = 0u;
                if (arc_index == 0u && sample_index == 0u)
                {
                    flags |= static_cast<uint32_t>(KeplerOrbitLineVertexFlags::OrbitStart);
                }
                if (arc_index + 1u == request.arcs.size() && sample_index + 1u == samples.size())
                {
                    flags |= static_cast<uint32_t>(KeplerOrbitLineVertexFlags::OrbitEnd);
                }
                if (sample_index == 0u)
                {
                    flags |= static_cast<uint32_t>(KeplerOrbitLineVertexFlags::ArcStart);
                }
                if (sample_index + 1u == samples.size())
                {
                    flags |= static_cast<uint32_t>(KeplerOrbitLineVertexFlags::ArcEnd);
                }

                const orbitsim::Vec3 inertial_position_m =
                        primary_state.position_m + sample.state_relative.position_m;
                const WorldVec3 world_position =
                        request.world_frame.world_reference_body_world +
                        WorldVec3(inertial_position_m - reference_state.position_m);
                append_vertex(out,
                              KeplerOrbitLineVertex{
                                      .position_world = world_position,
                                      .t_s = sample.t_s,
                                      .primary_body_id = game_arc.arc.primary_body_id,
                                      .flags = flags,
                              });
            }

            ++out.diagnostics.sampled_arcs;
        }

        if (out.vertices.size() < 2u)
        {
            out.diagnostics.status = KeplerOrbitStatus::NoSamples;
            return out;
        }

        out.valid = true;
        out.diagnostics.status = KeplerOrbitStatus::Ok;
        return out;
    }
} // namespace Game
