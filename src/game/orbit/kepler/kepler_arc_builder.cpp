#include "game/orbit/kepler/kepler_arc_builder.h"

#include "orbitsim/kepler_maneuver.hpp"
#include "orbitsim/math.hpp"

#include <cmath>
#include <vector>

namespace Game
{
    namespace
    {
        // Treat t1 as exclusive.
        bool time_in_arc_half_open(const double t_s, const double t0_s, const double t1_s)
        {
            if (t1_s >= t0_s)
            {
                return t_s >= t0_s && t_s < t1_s;
            }
            return t_s <= t0_s && t_s > t1_s;
        }
    } // namespace

    double select_kepler_arc_horizon_s(const orbitsim::KeplerArc &arc,
                                       const KeplerPredictionOptions &options)
    {
        // Ellipse: period-based. Open arc: time-window based.
        if (!orbitsim::kepler_arc_valid(arc))
        {
            return kepler_positive_or_default(options.open_orbit_window_s, 30.0 * 24.0 * 60.0 * 60.0);
        }

        const orbitsim::OrbitalElements elements =
                orbitsim::orbital_elements_from_relative_state(arc.mu_m3_s2,
                                                               arc.state0_relative.position_m,
                                                               arc.state0_relative.velocity_mps);
        const orbitsim::OrbitScalars scalars =
                orbitsim::orbit_scalars_from_elements(arc.mu_m3_s2, elements);

        if (scalars.valid &&
            scalars.period_s > 0.0 &&
            elements.eccentricity < 1.0)
        {
            const double period_count = kepler_positive_or_default(options.elliptic_period_count, 1.0);
            return scalars.period_s * period_count;
        }

        return kepler_positive_or_default(options.open_orbit_window_s, 30.0 * 24.0 * 60.0 * 60.0);
    }

    KeplerArcBuildResult build_kepler_arc(const KeplerArcBuildRequest &request)
    {
        KeplerArcBuildResult out{};
        if (!request.simulation)
        {
            out.status = KeplerOrbitStatus::InvalidSimulation;
            return out;
        }
        if (!kepler_finite_vec3(request.subject_state_inertial.position_m) ||
            !kepler_finite_vec3(request.subject_state_inertial.velocity_mps) ||
            !std::isfinite(request.t0_s))
        {
            out.status = KeplerOrbitStatus::InvalidSubjectState;
            return out;
        }

        // Resolve the primary at the arc start.
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

        // Kepler propagation starts from primary-relative state.
        orbitsim::State relative_state = request.subject_state_inertial;
        relative_state.position_m -= out.primary.state_inertial.position_m;
        relative_state.velocity_mps -= out.primary.state_inertial.velocity_mps;
        if (!(glm::length(relative_state.position_m) > 0.0))
        {
            out.status = KeplerOrbitStatus::InvalidSubjectState;
            return out;
        }

        out.base_arc.arc.mu_m3_s2 = out.primary.mu_m3_s2;
        out.base_arc.arc.primary_body_id = out.primary.body_id;
        out.base_arc.arc.t0_s = request.t0_s;
        out.base_arc.arc.state0_relative = relative_state;
        out.base_arc.primary_state_inertial_at_t0 = out.primary.state_inertial;

        // Line tessellation samples between t0 and t1.
        out.horizon_s = (request.requested_horizon_s > 0.0)
                                ? request.requested_horizon_s
                                : select_kepler_arc_horizon_s(out.base_arc.arc, request.options);
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

    KeplerManeuverArcBuildResult build_kepler_maneuver_arc_chain(
            const KeplerOrbitArc &base_arc,
            const std::span<const KeplerManeuverNode> nodes,
            const orbitsim::KeplerPropagationOptions &propagation_options)
    {
        KeplerManeuverArcBuildResult out{};
        if (!orbitsim::kepler_arc_valid(base_arc.arc))
        {
            out.status = KeplerOrbitStatus::InvalidArc;
            return out;
        }

        std::vector<orbitsim::KeplerImpulse> impulses;
        impulses.reserve(nodes.size());
        for (const KeplerManeuverNode &node : nodes)
        {
            // Keep impulses this arc can own.
            if (!std::isfinite(node.t_s) || !kepler_finite_vec3(node.dv_rtn_mps))
            {
                out.status = KeplerOrbitStatus::InvalidInput;
                return out;
            }
            if (!time_in_arc_half_open(node.t_s, base_arc.arc.t0_s, base_arc.arc.t1_s))
            {
                continue;
            }
            if (node.primary_body_id != orbitsim::kInvalidBodyId &&
                node.primary_body_id != base_arc.arc.primary_body_id)
            {
                out.status = KeplerOrbitStatus::PrimaryMismatch;
                return out;
            }

            impulses.push_back(orbitsim::KeplerImpulse{
                    .t_s = node.t_s,
                    .dv_rtn_mps = node.dv_rtn_mps,
            });
        }

        const std::vector<orbitsim::KeplerArc> solved_arcs =
                orbitsim::build_maneuvered_kepler_arcs(base_arc.arc,
                                                       impulses,
                                                       propagation_options,
                                                       &out.diagnostics);
        if (solved_arcs.empty())
        {
            out.status = out.diagnostics.first_failure == orbitsim::KeplerStatus::Ok
                                 ? KeplerOrbitStatus::InvalidArc
                                 : KeplerOrbitStatus::PropagationFailed;
            return out;
        }
        if (out.diagnostics.first_failure != orbitsim::KeplerStatus::Ok)
        {
            out.status = KeplerOrbitStatus::PropagationFailed;
            return out;
        }

        out.arcs.reserve(solved_arcs.size());
        for (const orbitsim::KeplerArc &arc : solved_arcs)
        {
            out.arcs.push_back(KeplerOrbitArc{
                    .arc = arc,
                    .primary_state_inertial_at_t0 = base_arc.primary_state_inertial_at_t0,
            });
        }

        out.valid = true;
        out.status = KeplerOrbitStatus::Ok;
        return out;
    }
} // namespace Game
