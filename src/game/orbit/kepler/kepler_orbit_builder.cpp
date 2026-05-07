#include "game/orbit/kepler/kepler_orbit_builder.h"

#include "orbitsim/math.hpp"

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

        double positive_or_default(const double value, const double fallback)
        {
            return (std::isfinite(value) && value > 0.0) ? value : fallback;
        }
    } // namespace

    double select_kepler_horizon_s(const orbitsim::KeplerArc &arc,
                                   const KeplerPredictionOptions &options)
    {
        if (!orbitsim::kepler_arc_valid(arc))
        {
            return positive_or_default(options.open_orbit_window_s, 30.0 * 24.0 * 60.0 * 60.0);
        }

        const orbitsim::OrbitalElements elements =
                orbitsim::orbital_elements_from_relative_state(arc.mu_m3_s2,
                                                               arc.state0_relative.position_m,
                                                               arc.state0_relative.velocity_mps);
        const orbitsim::OrbitScalars scalars =
                orbitsim::orbit_scalars_from_elements(arc.mu_m3_s2, elements);

        if (scalars.valid &&
            scalars.period_s > 0.0 &&
            std::isfinite(scalars.period_s) &&
            elements.eccentricity < 1.0)
        {
            const double period_count = positive_or_default(options.elliptic_period_count, 1.0);
            return scalars.period_s * period_count;
        }

        return positive_or_default(options.open_orbit_window_s, 30.0 * 24.0 * 60.0 * 60.0);
    }

    KeplerOrbitBuildResult build_kepler_orbit(const KeplerOrbitBuildRequest &request)
    {
        KeplerOrbitBuildResult out{};
        if (!request.simulation)
        {
            out.status = KeplerOrbitStatus::InvalidSimulation;
            return out;
        }
        if (!finite_state(request.subject_state_inertial) || !std::isfinite(request.t0_s))
        {
            out.status = KeplerOrbitStatus::InvalidSubjectState;
            return out;
        }

        KeplerPrimaryResolveRequest primary_request{};
        primary_request.simulation = request.simulation;
        primary_request.ephemeris = request.ephemeris;
        primary_request.query_position_m = request.subject_state_inertial.position_m;
        primary_request.query_time_s = request.t0_s;
        primary_request.fixed_primary_body_id = request.fixed_primary_body_id;
        primary_request.current_primary_body_id = request.current_primary_body_id;
        primary_request.switch_options = request.options.primary_switch;
        primary_request.fallback_primary_hysteresis_keep_ratio =
                request.options.fallback_primary_hysteresis_keep_ratio;

        out.primary = resolve_kepler_primary(primary_request);
        if (!out.primary.valid)
        {
            out.status = out.primary.status;
            return out;
        }

        orbitsim::State relative_state = request.subject_state_inertial;
        relative_state.position_m -= out.primary.state_inertial.position_m;
        relative_state.velocity_mps -= out.primary.state_inertial.velocity_mps;
        if (!finite_state(relative_state) || !(glm::length(relative_state.position_m) > 0.0))
        {
            out.status = KeplerOrbitStatus::InvalidSubjectState;
            return out;
        }

        out.base_arc.arc.mu_m3_s2 = out.primary.mu_m3_s2;
        out.base_arc.arc.primary_body_id = out.primary.body_id;
        out.base_arc.arc.t0_s = request.t0_s;
        out.base_arc.arc.state0_relative = relative_state;
        out.base_arc.primary_state_inertial_at_t0 = out.primary.state_inertial;

        out.horizon_s = (request.requested_horizon_s > 0.0 && std::isfinite(request.requested_horizon_s))
                                ? request.requested_horizon_s
                                : select_kepler_horizon_s(out.base_arc.arc, request.options);
        out.base_arc.arc.t1_s = request.t0_s + out.horizon_s;

        if (!orbitsim::kepler_arc_valid(out.base_arc.arc))
        {
            out.status = KeplerOrbitStatus::InvalidArc;
            return out;
        }

        out.valid = true;
        out.status = KeplerOrbitStatus::Ok;
        return out;
    }
} // namespace Game
