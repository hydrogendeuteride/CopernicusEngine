#include "game/orbit/kepler/kepler_primary_resolver.h"

#include "orbitsim/soi.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

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

        bool resolve_body_state(const orbitsim::GameSimulation &sim,
                                const orbitsim::CelestialEphemeris *ephemeris,
                                const orbitsim::BodyId body_id,
                                const double t_s,
                                orbitsim::State &out_state)
        {
            const orbitsim::MassiveBody *body = sim.body_by_id(body_id);
            if (!body)
            {
                return false;
            }

            if (ephemeris && !ephemeris->empty())
            {
                out_state = ephemeris->body_state_at_by_id(body_id, t_s);
            }
            else
            {
                out_state = body->state;
            }
            return finite_state(out_state);
        }

        KeplerPrimaryResolution make_resolution(const orbitsim::GameSimulation &sim,
                                                const orbitsim::CelestialEphemeris *ephemeris,
                                                const orbitsim::BodyId body_id,
                                                const double t_s)
        {
            KeplerPrimaryResolution out{};
            const orbitsim::MassiveBody *body = sim.body_by_id(body_id);
            if (!body)
            {
                out.status = KeplerOrbitStatus::PrimaryUnavailable;
                return out;
            }

            const double g = sim.config().gravitational_constant;
            const double mu = g * body->mass_kg;
            if (!(g > 0.0) || !std::isfinite(g) ||
                !(body->mass_kg > 0.0) || !std::isfinite(body->mass_kg) ||
                !(mu > 0.0) || !std::isfinite(mu))
            {
                out.status = KeplerOrbitStatus::InvalidPrimary;
                return out;
            }

            orbitsim::State state{};
            if (!resolve_body_state(sim, ephemeris, body_id, t_s, state))
            {
                out.status = KeplerOrbitStatus::InvalidPrimary;
                return out;
            }

            out.valid = true;
            out.status = KeplerOrbitStatus::Ok;
            out.body_id = body_id;
            out.mass_kg = body->mass_kg;
            out.radius_m = body->radius_m;
            out.mu_m3_s2 = mu;
            out.state_inertial = state;
            return out;
        }

        std::optional<double> gravity_metric_at(const orbitsim::GameSimulation &sim,
                                                const orbitsim::CelestialEphemeris &ephemeris,
                                                const orbitsim::BodyId body_id,
                                                const orbitsim::Vec3 &query_position_m,
                                                const double query_time_s)
        {
            const orbitsim::MassiveBody *body = sim.body_by_id(body_id);
            if (!body || !(body->mass_kg > 0.0) || !std::isfinite(body->mass_kg))
            {
                return std::nullopt;
            }

            orbitsim::Vec3 body_position_m{0.0, 0.0, 0.0};
            if (!ephemeris.empty())
            {
                body_position_m = ephemeris.body_position_at_by_id(body_id, query_time_s);
            }
            else
            {
                body_position_m = body->state.position_m;
            }
            if (!finite_vec3(body_position_m))
            {
                return std::nullopt;
            }

            const double eps2 = sim.config().softening_length_m * sim.config().softening_length_m;
            const orbitsim::Vec3 dr = body_position_m - query_position_m;
            const double r2 = glm::dot(dr, dr) + eps2;
            if (!(r2 > 0.0) || !std::isfinite(r2))
            {
                return std::nullopt;
            }
            return body->mass_kg / r2;
        }

        orbitsim::BodyId select_primary_by_max_accel_with_hysteresis(
                const orbitsim::GameSimulation &sim,
                const orbitsim::CelestialEphemeris &ephemeris,
                const orbitsim::Vec3 &query_position_m,
                const double query_time_s,
                const orbitsim::BodyId current_primary_body_id,
                const double hysteresis_keep_ratio)
        {
            orbitsim::BodyId best_body_id = orbitsim::kInvalidBodyId;
            double best_metric = -1.0;
            std::optional<double> current_metric{};

            for (const orbitsim::MassiveBody &body : sim.massive_bodies())
            {
                const std::optional<double> metric =
                        gravity_metric_at(sim, ephemeris, body.id, query_position_m, query_time_s);
                if (!metric.has_value())
                {
                    continue;
                }

                if (*metric > best_metric)
                {
                    best_metric = *metric;
                    best_body_id = body.id;
                }

                if (body.id == current_primary_body_id)
                {
                    current_metric = *metric;
                }
            }

            if (best_body_id == orbitsim::kInvalidBodyId || !(best_metric > 0.0))
            {
                return orbitsim::kInvalidBodyId;
            }

            const double keep_ratio =
                    std::isfinite(hysteresis_keep_ratio)
                            ? std::clamp(hysteresis_keep_ratio, 0.0, 1.0)
                            : 0.0;
            if (current_primary_body_id != orbitsim::kInvalidBodyId &&
                current_metric.has_value() &&
                *current_metric > 0.0 &&
                *current_metric >= best_metric * keep_ratio)
            {
                return current_primary_body_id;
            }

            return best_body_id;
        }
    } // namespace

    KeplerPrimaryResolution resolve_kepler_primary(const KeplerPrimaryResolveRequest &request)
    {
        KeplerPrimaryResolution out{};
        if (!request.simulation)
        {
            out.status = KeplerOrbitStatus::InvalidSimulation;
            return out;
        }
        if (!finite_vec3(request.query_position_m) || !std::isfinite(request.query_time_s))
        {
            out.status = KeplerOrbitStatus::InvalidInput;
            return out;
        }

        const orbitsim::GameSimulation &sim = *request.simulation;
        if (sim.massive_bodies().empty())
        {
            out.status = KeplerOrbitStatus::PrimaryUnavailable;
            return out;
        }

        if (request.fixed_primary_body_id != orbitsim::kInvalidBodyId)
        {
            return make_resolution(sim, request.ephemeris, request.fixed_primary_body_id, request.query_time_s);
        }

        const orbitsim::CelestialEphemeris empty_ephemeris{};
        const orbitsim::CelestialEphemeris &ephemeris =
                request.ephemeris ? *request.ephemeris : empty_ephemeris;
        orbitsim::SoiSwitchOptions soi_only_options = request.switch_options;
        soi_only_options.fallback_to_max_accel = false;

        const orbitsim::BodyId selected_body_id =
                orbitsim::select_primary_body_id_rails(sim,
                                                       ephemeris,
                                                       request.query_position_m,
                                                       request.query_time_s,
                                                       request.current_primary_body_id,
                                                       soi_only_options);
        if (selected_body_id != orbitsim::kInvalidBodyId)
        {
            return make_resolution(sim, request.ephemeris, selected_body_id, request.query_time_s);
        }

        if (!request.switch_options.fallback_to_max_accel)
        {
            out.status = KeplerOrbitStatus::PrimaryUnavailable;
            return out;
        }

        const orbitsim::BodyId fallback_body_id =
                select_primary_by_max_accel_with_hysteresis(sim,
                                                            ephemeris,
                                                            request.query_position_m,
                                                            request.query_time_s,
                                                            request.current_primary_body_id,
                                                            request.fallback_primary_hysteresis_keep_ratio);
        if (fallback_body_id == orbitsim::kInvalidBodyId)
        {
            out.status = KeplerOrbitStatus::PrimaryUnavailable;
            return out;
        }

        return make_resolution(sim, request.ephemeris, fallback_body_id, request.query_time_s);
    }
} // namespace Game
