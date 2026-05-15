#include "game/orbit/kepler/kepler_celestial_nbody.h"

#include "orbitsim/trajectories.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace Game
{
    namespace
    {
        orbitsim::AdaptiveEphemerisOptions to_orbitsim_ephemeris_options(
                const double horizon_s,
                const KeplerCelestialNBodyEphemerisOptions &options)
        {
            const std::size_t soft_max_segments =
                    options.soft_max_segments > 0u ? options.soft_max_segments : 3000u;
            const std::size_t configured_hard_max_segments =
                    options.hard_max_segments > 0u ? options.hard_max_segments : soft_max_segments;
            const double pos_tolerance_m = kepler_positive_or_default(options.pos_tolerance_m, 10.0);
            const double vel_tolerance_mps = kepler_positive_or_default(options.vel_tolerance_mps, 1.0);
            const double relative_tolerance_floor =
                    kepler_positive_or_default(options.relative_tolerance_floor, 1.0e-7);

            orbitsim::AdaptiveEphemerisOptions out{
                    .duration_s = horizon_s,
                    .min_dt_s = kepler_positive_or_default(options.min_dt_s, 1.0),
                    .max_dt_s = kepler_positive_or_default(options.max_dt_s, 600.0),
                    .soft_max_segments = soft_max_segments,
                    .hard_max_segments = std::max(soft_max_segments, configured_hard_max_segments),
                    .tolerance = orbitsim::AdaptiveToleranceRamp{
                            .pos_near_m = pos_tolerance_m,
                            .pos_far_m = pos_tolerance_m,
                            .vel_near_mps = vel_tolerance_mps,
                            .vel_far_mps = vel_tolerance_mps,
                            .rel_pos_floor = relative_tolerance_floor,
                            .rel_vel_floor = relative_tolerance_floor,
                    },
            };
            out.max_dt_s = std::max(out.min_dt_s, out.max_dt_s);

            // Widen max step so long horizons stay near the soft segment budget.
            const double min_max_dt_for_soft_cap =
                    horizon_s / static_cast<double>(std::max<std::size_t>(soft_max_segments, 1u));
            if (min_max_dt_for_soft_cap > 0.0)
            {
                out.max_dt_s = std::max(out.max_dt_s, min_max_dt_for_soft_cap);
            }

            return out;
        }

        // Detached copy preserves body ids without mutating live simulation.
        bool clone_massive_body_simulation_at(const orbitsim::GameSimulation &source,
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
    } // namespace

    KeplerCelestialNBodyHorizonLimit limit_kepler_celestial_nbody_horizon(
            const double horizon_s,
            const KeplerPredictionOptions &options)
    {
        return limit_kepler_celestial_nbody_horizon(horizon_s, options, 0.0);
    }

    KeplerCelestialNBodyHorizonLimit limit_kepler_celestial_nbody_horizon(
            const double horizon_s,
            const KeplerPredictionOptions &options,
            const double horizon_floor_s)
    {
        KeplerCelestialNBodyHorizonLimit out{};
        out.uncapped_horizon_s = kepler_positive_or_default(horizon_s, 0.0);
        out.horizon_s = out.uncapped_horizon_s;
        out.cap_s = kepler_positive_or_default(options.celestial_nbody_horizon_cap_s, 0.0);
        const double floor_s = kepler_positive_or_default(horizon_floor_s, 0.0);

        if (out.cap_s > 0.0)
        {
            const double effective_cap_s = std::max(out.cap_s, floor_s);
            if (out.horizon_s > effective_cap_s)
            {
                out.horizon_s = effective_cap_s;
                out.capped = true;
            }
        }
        if (floor_s > out.horizon_s)
        {
            out.horizon_s = floor_s;
        }
        return out;
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

        out.horizon_s =
                (std::isfinite(request.requested_horizon_s) && request.requested_horizon_s > 0.0)
                        ? request.requested_horizon_s
                        : kepler_positive_or_default(request.options.open_orbit_window_s,
                                                     24.0 * 60.0 * 60.0);
        if (!(out.horizon_s > 0.0) || !std::isfinite(out.horizon_s))
        {
            out.status = KeplerOrbitStatus::InvalidInput;
            return out;
        }

        orbitsim::GameSimulation ephemeris_sim{};
        if (!clone_massive_body_simulation_at(*request.simulation, request.t0_s, ephemeris_sim))
        {
            out.status = KeplerOrbitStatus::InvalidSimulation;
            return out;
        }

        const orbitsim::AdaptiveEphemerisOptions adaptive_options =
                to_orbitsim_ephemeris_options(out.horizon_s,
                                              request.options.celestial_nbody_ephemeris);

        // Actual celestial n-body integration.
        auto ephemeris = std::make_shared<orbitsim::CelestialEphemeris>(
                orbitsim::build_celestial_ephemeris_adaptive(ephemeris_sim,
                                                             adaptive_options,
                                                             &out.diagnostics));
        if (!ephemeris || ephemeris->empty())
        {
            out.status = KeplerOrbitStatus::EphemerisUnavailable;
            return out;
        }

        out.valid = true;
        out.status = KeplerOrbitStatus::Ok;
        out.ephemeris = std::move(ephemeris);
        out.body_state_provider.state_at =
                [ephemeris = out.ephemeris, fallback_simulation = request.simulation](
                        const orbitsim::BodyId body_id,
                        const double t_s,
                        orbitsim::State &out_state) {
                    // Prefer ephemeris so arc rendering follows moving bodies.
                    if (ephemeris && !ephemeris->empty())
                    {
                        std::size_t body_index = 0u;
                        if (ephemeris->body_index_for_id(body_id, &body_index))
                        {
                            out_state = ephemeris->body_state_at(body_index, t_s);
                            return true;
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
                    return true;
                };
        return out;
    }

    KeplerArcLineSet build_kepler_celestial_nbody_lines(
            const KeplerCelestialNBodyLineRequest &request)
    {
        KeplerArcLineSet out{};
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

        const bool use_reference_ephemeris =
                request.world_frame.world_reference_body_id != orbitsim::kInvalidBodyId;
        std::size_t reference_index = 0u;
        if (use_reference_ephemeris &&
            !request.ephemeris->body_index_for_id(request.world_frame.world_reference_body_id,
                                                  &reference_index))
        {
            out.diagnostics.status = KeplerOrbitStatus::InvalidPrimary;
            return out;
        }

        const double fallback_line_horizon_s =
                std::max(0.0, request.ephemeris->t_end_s() - request.t0_s);
        const double body_horizon_s =
                kepler_positive_or_default(request.requested_horizon_s, fallback_line_horizon_s);
        const double t1_s = std::min(request.t0_s + body_horizon_s, request.ephemeris->t_end_s());
        const double duration_s = t1_s - request.t0_s;
        if (!(duration_s > 0.0))
        {
            out.diagnostics.status = KeplerOrbitStatus::NoSamples;
            return out;
        }

        const std::size_t allowed_samples =
                std::min(request.line_options.max_vertices_per_arc,
                         request.line_options.max_vertices_total);
        if (allowed_samples < 2u)
        {
            out.diagnostics.status = KeplerOrbitStatus::SampleBudgetExceeded;
            out.diagnostics.budget_hit = true;
            return out;
        }

        // Widen dt when the vertex budget cannot cover the full horizon.
        const double configured_dt =
                kepler_positive_or_default(request.line_options.max_time_step_s, 60.0);
        const double min_dt =
                kepler_positive_or_default(request.line_options.min_time_step_s, 1.0);
        const double budget_dt_s =
                duration_s / static_cast<double>(allowed_samples - 1u);
        const double dt_s = std::max({min_dt, configured_dt, budget_dt_s});
        const double raw_sample_count = std::ceil(duration_s / dt_s) + 1.0;
        const std::size_t sample_count =
                (std::isfinite(raw_sample_count) && raw_sample_count >= 2.0)
                        ? std::clamp<std::size_t>(static_cast<std::size_t>(raw_sample_count),
                                                  2u,
                                                  allowed_samples)
                        : 0u;
        if (sample_count < 2u)
        {
            out.diagnostics.status = KeplerOrbitStatus::NoSamples;
            return out;
        }

        if (sample_count == allowed_samples && dt_s > request.line_options.max_time_step_s)
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
            if (!kepler_finite_vec3(body_state.position_m))
            {
                out.diagnostics.status = KeplerOrbitStatus::PropagationFailed;
                out.diagnostics.failed_arc_index = 0u;
                return out;
            }

            orbitsim::State reference_state = request.world_frame.world_reference_state_inertial;
            if (use_reference_ephemeris)
            {
                reference_state = request.ephemeris->body_state_at(reference_index, t_s);
            }
            if (!kepler_finite_vec3(reference_state.position_m))
            {
                out.diagnostics.status = KeplerOrbitStatus::InvalidPrimary;
                return out;
            }

            // Convert inertial ephemeris position into gameplay world space.
            uint32_t flags = 0u;
            if (sample_index == 0u)
            {
                flags |= static_cast<uint32_t>(KeplerArcLineVertexFlags::OrbitStart);
                flags |= static_cast<uint32_t>(KeplerArcLineVertexFlags::ArcStart);
            }
            if (sample_index + 1u == sample_count)
            {
                flags |= static_cast<uint32_t>(KeplerArcLineVertexFlags::OrbitEnd);
                flags |= static_cast<uint32_t>(KeplerArcLineVertexFlags::ArcEnd);
            }

            const WorldVec3 world_position =
                    request.world_frame.world_reference_body_world +
                    WorldVec3(body_state.position_m - reference_state.position_m);
            out.vertices.push_back(KeplerArcLineVertex{
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
