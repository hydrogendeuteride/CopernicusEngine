#include "game/orbit/kepler/kepler_celestial_nbody.h"

#include "orbitsim/trajectories.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

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

        double positive_or_default(const double value, const double fallback)
        {
            return (std::isfinite(value) && value > 0.0) ? value : fallback;
        }

        double fallback_horizon_s(const KeplerPredictionOptions &options)
        {
            return positive_or_default(options.open_orbit_window_s, 24.0 * 60.0 * 60.0);
        }

        double configured_horizon_cap_s(const KeplerPredictionOptions &options)
        {
            return positive_or_default(options.celestial_nbody_horizon_cap_s, 0.0);
        }

        std::size_t positive_size_or_default(const std::size_t value,
                                             const std::size_t fallback)
        {
            return value > 0u ? value : fallback;
        }

        orbitsim::AdaptiveEphemerisOptions build_adaptive_ephemeris_options(
                const double horizon_s,
                const KeplerCelestialNBodyEphemerisOptions &options)
        {
            orbitsim::AdaptiveEphemerisOptions out{};
            out.duration_s = horizon_s;
            out.min_dt_s = positive_or_default(options.min_dt_s, 1.0);
            out.max_dt_s = std::max(out.min_dt_s, positive_or_default(options.max_dt_s, 600.0));

            const std::size_t soft_max_segments =
                    positive_size_or_default(options.soft_max_segments, 3000u);
            const std::size_t hard_max_segments =
                    std::max(soft_max_segments,
                             positive_size_or_default(options.hard_max_segments, soft_max_segments));
            const double min_max_dt_for_soft_cap =
                    horizon_s / static_cast<double>(std::max<std::size_t>(soft_max_segments, 1u));
            if (std::isfinite(min_max_dt_for_soft_cap) && min_max_dt_for_soft_cap > 0.0)
            {
                out.max_dt_s = std::max(out.max_dt_s, min_max_dt_for_soft_cap);
            }

            out.soft_max_segments = soft_max_segments;
            out.hard_max_segments = hard_max_segments;
            const double pos_tolerance_m = positive_or_default(options.pos_tolerance_m, 10.0);
            const double vel_tolerance_mps = positive_or_default(options.vel_tolerance_mps, 1.0);
            const double relative_tolerance_floor =
                    positive_or_default(options.relative_tolerance_floor, 1.0e-7);
            out.tolerance.pos_near_m = pos_tolerance_m;
            out.tolerance.pos_far_m = pos_tolerance_m;
            out.tolerance.vel_near_mps = vel_tolerance_mps;
            out.tolerance.vel_far_mps = vel_tolerance_mps;
            out.tolerance.rel_pos_floor = relative_tolerance_floor;
            out.tolerance.rel_vel_floor = relative_tolerance_floor;
            return out;
        }

        bool build_ephemeris_simulation(const orbitsim::GameSimulation &source,
                                        const double t0_s,
                                        orbitsim::GameSimulation &out_simulation)
        {
            out_simulation = orbitsim::GameSimulation(source.config());
            if (!out_simulation.set_time_s(t0_s))
            {
                return false;
            }

            for (const orbitsim::MassiveBody &body : source.massive_bodies())
            {
                const auto handle =
                        (body.id != orbitsim::kInvalidBodyId)
                                ? out_simulation.create_body_with_id(body.id, body)
                                : out_simulation.create_body(body);
                if (!handle.valid())
                {
                    return false;
                }
            }
            return true;
        }

        bool validate_ephemeris_shape(const orbitsim::CelestialEphemeris &ephemeris)
        {
            if (ephemeris.empty() || ephemeris.body_ids.empty())
            {
                return false;
            }

            for (std::size_t segment_index = 0; segment_index < ephemeris.segments.size(); ++segment_index)
            {
                const orbitsim::CelestialEphemerisSegment &segment = ephemeris.segments[segment_index];
                if (!(segment.dt_s > 0.0) ||
                    !std::isfinite(segment.t0_s) ||
                    !std::isfinite(segment.dt_s) ||
                    segment.start.size() != ephemeris.body_ids.size() ||
                    segment.end.size() != ephemeris.body_ids.size())
                {
                    return false;
                }

                if (segment_index > 0u)
                {
                    const orbitsim::CelestialEphemerisSegment &prev =
                            ephemeris.segments[segment_index - 1u];
                    const double prev_t1_s = prev.t0_s + prev.dt_s;
                    if (std::abs(segment.t0_s - prev_t1_s) > 1.0e-6)
                    {
                        return false;
                    }
                }

                for (std::size_t body_index = 0; body_index < segment.start.size(); ++body_index)
                {
                    if (!finite_state(segment.start[body_index]) ||
                        !finite_state(segment.end[body_index]))
                    {
                        return false;
                    }
                }
            }

            return true;
        }

        std::size_t allowed_line_samples(const KeplerOrbitTessellationOptions &options)
        {
            return std::min(options.max_vertices_per_arc, options.max_vertices_total);
        }

        double sample_dt_s(const double duration_s,
                           const KeplerOrbitTessellationOptions &options,
                           const std::size_t allowed_samples)
        {
            const double configured_dt =
                    (std::isfinite(options.max_time_step_s) && options.max_time_step_s > 0.0)
                            ? options.max_time_step_s
                            : 60.0;
            const double min_dt =
                    (std::isfinite(options.min_time_step_s) && options.min_time_step_s > 0.0)
                            ? options.min_time_step_s
                            : 1.0;
            double dt_s = std::max(min_dt, configured_dt);
            if (allowed_samples > 1u && duration_s > 0.0 && std::isfinite(duration_s))
            {
                const double budget_dt_s = duration_s / static_cast<double>(allowed_samples - 1u);
                if (std::isfinite(budget_dt_s) && budget_dt_s > dt_s)
                {
                    dt_s = budget_dt_s;
                }
            }
            return dt_s;
        }

        std::size_t sample_count_for(const double duration_s,
                                     const double dt_s,
                                     const std::size_t allowed_samples)
        {
            if (!(duration_s > 0.0) || !(dt_s > 0.0) || allowed_samples < 2u)
            {
                return 0u;
            }

            const double raw_count = std::ceil(duration_s / dt_s) + 1.0;
            if (!std::isfinite(raw_count) || raw_count < 2.0)
            {
                return 0u;
            }
            return std::clamp<std::size_t>(static_cast<std::size_t>(raw_count), 2u, allowed_samples);
        }

        bool same_sample_time(const double a, const double b)
        {
            return std::isfinite(a) && std::isfinite(b) && std::abs(a - b) <= 1.0e-9;
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

    double select_kepler_celestial_nbody_ephemeris_horizon_s(
            const KeplerCelestialNBodyEphemerisRequest &request)
    {
        if (std::isfinite(request.requested_horizon_s) && request.requested_horizon_s > 0.0)
        {
            return request.requested_horizon_s;
        }
        if (!request.simulation)
        {
            return fallback_horizon_s(request.options);
        }

        return fallback_horizon_s(request.options);
    }

    KeplerCelestialNBodyHorizonLimit limit_kepler_celestial_nbody_horizon(
            const double horizon_s,
            const KeplerPredictionOptions &options)
    {
        KeplerCelestialNBodyHorizonLimit out{};
        out.uncapped_horizon_s = positive_or_default(horizon_s, 0.0);
        out.horizon_s = out.uncapped_horizon_s;
        out.cap_s = configured_horizon_cap_s(options);

        if (out.cap_s > 0.0 &&
            std::isfinite(out.cap_s) &&
            out.horizon_s > out.cap_s)
        {
            out.horizon_s = out.cap_s;
            out.capped = true;
        }
        return out;
    }

    KeplerBodyStateProvider make_kepler_celestial_nbody_state_provider(
            KeplerSharedCelestialEphemeris ephemeris,
            const orbitsim::GameSimulation *fallback_simulation)
    {
        KeplerBodyStateProvider provider{};
        provider.state_at = [ephemeris = std::move(ephemeris), fallback_simulation](
                                    const orbitsim::BodyId body_id,
                                    const double t_s,
                                    orbitsim::State &out_state) {
            if (ephemeris && !ephemeris->empty())
            {
                std::size_t body_index = 0u;
                if (ephemeris->body_index_for_id(body_id, &body_index))
                {
                    out_state = ephemeris->body_state_at(body_index, t_s);
                    if (finite_state(out_state))
                    {
                        return true;
                    }
                }
            }

            if (!fallback_simulation)
            {
                return false;
            }
            const orbitsim::MassiveBody *body = fallback_simulation->body_by_id(body_id);
            if (!body)
            {
                return false;
            }
            out_state = body->state;
            return finite_state(out_state);
        };
        return provider;
    }

    KeplerCelestialNBodyEphemerisResult build_kepler_celestial_nbody_ephemeris(
            const KeplerCelestialNBodyEphemerisRequest &request)
    {
        KeplerCelestialNBodyEphemerisResult out{};
        if (!request.simulation ||
            !std::isfinite(request.t0_s) ||
            request.simulation->massive_bodies().empty())
        {
            out.status = KeplerOrbitStatus::InvalidSimulation;
            return out;
        }

        out.horizon_s = select_kepler_celestial_nbody_ephemeris_horizon_s(request);
        if (!(out.horizon_s > 0.0) || !std::isfinite(out.horizon_s))
        {
            out.status = KeplerOrbitStatus::InvalidInput;
            return out;
        }

        orbitsim::GameSimulation ephemeris_sim{};
        if (!build_ephemeris_simulation(*request.simulation, request.t0_s, ephemeris_sim))
        {
            out.status = KeplerOrbitStatus::InvalidSimulation;
            return out;
        }

        orbitsim::AdaptiveEphemerisOptions adaptive_options =
                build_adaptive_ephemeris_options(out.horizon_s,
                                                 request.options.celestial_nbody_ephemeris);
        auto ephemeris = std::make_shared<orbitsim::CelestialEphemeris>(
                orbitsim::build_celestial_ephemeris_adaptive(ephemeris_sim,
                                                             adaptive_options,
                                                             &out.diagnostics));
        if (!ephemeris || ephemeris->empty())
        {
            out.status = KeplerOrbitStatus::EphemerisUnavailable;
            return out;
        }
        if (!validate_ephemeris_shape(*ephemeris))
        {
            out.status = KeplerOrbitStatus::ContinuityFailed;
            return out;
        }

        out.valid = true;
        out.status = KeplerOrbitStatus::Ok;
        out.ephemeris = std::move(ephemeris);
        out.body_state_provider =
                make_kepler_celestial_nbody_state_provider(out.ephemeris, request.simulation);
        return out;
    }

    KeplerOrbitLineSet build_kepler_celestial_nbody_lines(
            const KeplerCelestialNBodyLineRequest &request)
    {
        KeplerOrbitLineSet out{};
        out.diagnostics.requested_arcs = 1u;

        if (!request.ephemeris ||
            request.ephemeris->empty() ||
            request.body_id == orbitsim::kInvalidBodyId ||
            !std::isfinite(request.t0_s))
        {
            out.diagnostics.status = KeplerOrbitStatus::InvalidInput;
            return out;
        }

        std::size_t body_index = 0u;
        if (!request.ephemeris->body_index_for_id(request.body_id, &body_index))
        {
            out.diagnostics.status = KeplerOrbitStatus::InvalidInput;
            return out;
        }

        const double fallback_line_horizon_s =
                std::max(0.0, request.ephemeris->t_end_s() - request.t0_s);
        const double body_horizon_s =
                positive_or_default(request.requested_horizon_s, fallback_line_horizon_s);
        const double t1_s = std::min(request.t0_s + body_horizon_s, request.ephemeris->t_end_s());
        const double duration_s = t1_s - request.t0_s;
        if (!(duration_s > 0.0) || !std::isfinite(duration_s))
        {
            out.diagnostics.status = KeplerOrbitStatus::NoSamples;
            return out;
        }

        const std::size_t allowed_samples = allowed_line_samples(request.tessellation);
        if (allowed_samples < 2u)
        {
            out.diagnostics.status = KeplerOrbitStatus::SampleBudgetExceeded;
            out.diagnostics.budget_hit = true;
            return out;
        }

        const double dt_s = sample_dt_s(duration_s, request.tessellation, allowed_samples);
        const std::size_t sample_count = sample_count_for(duration_s, dt_s, allowed_samples);
        if (sample_count < 2u)
        {
            out.diagnostics.status = KeplerOrbitStatus::NoSamples;
            return out;
        }

        if (sample_count == allowed_samples && dt_s > request.tessellation.max_time_step_s)
        {
            out.diagnostics.budget_hit = true;
        }
        out.diagnostics.requested_samples = sample_count;

        for (std::size_t sample_index = 0u; sample_index < sample_count; ++sample_index)
        {
            const double t_s =
                    (sample_index + 1u == sample_count)
                            ? t1_s
                            : request.t0_s + static_cast<double>(sample_index) * dt_s;
            const orbitsim::State body_state = request.ephemeris->body_state_at(body_index, t_s);
            if (!finite_state(body_state))
            {
                out.diagnostics.status = KeplerOrbitStatus::PropagationFailed;
                out.diagnostics.failed_arc_index = 0u;
                return out;
            }

            orbitsim::State reference_state = request.world_frame.world_reference_state_inertial;
            if (request.world_frame.world_reference_body_id != orbitsim::kInvalidBodyId)
            {
                std::size_t reference_index = 0u;
                if (!request.ephemeris->body_index_for_id(request.world_frame.world_reference_body_id,
                                                          &reference_index))
                {
                    out.diagnostics.status = KeplerOrbitStatus::InvalidPrimary;
                    return out;
                }
                reference_state = request.ephemeris->body_state_at(reference_index, t_s);
            }
            if (!finite_state(reference_state))
            {
                out.diagnostics.status = KeplerOrbitStatus::InvalidPrimary;
                return out;
            }

            uint32_t flags = 0u;
            if (sample_index == 0u)
            {
                flags |= static_cast<uint32_t>(KeplerOrbitLineVertexFlags::OrbitStart);
                flags |= static_cast<uint32_t>(KeplerOrbitLineVertexFlags::ArcStart);
            }
            if (sample_index + 1u == sample_count)
            {
                flags |= static_cast<uint32_t>(KeplerOrbitLineVertexFlags::OrbitEnd);
                flags |= static_cast<uint32_t>(KeplerOrbitLineVertexFlags::ArcEnd);
            }

            const WorldVec3 world_position =
                    request.world_frame.world_reference_body_world +
                    WorldVec3(body_state.position_m - reference_state.position_m);
            append_vertex(out,
                          KeplerOrbitLineVertex{
                                  .position_world = world_position,
                                  .t_s = t_s,
                                  .primary_body_id = request.world_frame.world_reference_body_id,
                                  .flags = flags,
                          });
            ++out.diagnostics.accepted_samples;
        }

        if (out.vertices.size() < 2u)
        {
            out.diagnostics.status = KeplerOrbitStatus::NoSamples;
            return out;
        }

        out.valid = true;
        out.diagnostics.sampled_arcs = 1u;
        out.diagnostics.status = KeplerOrbitStatus::Ok;
        return out;
    }
} // namespace Game
