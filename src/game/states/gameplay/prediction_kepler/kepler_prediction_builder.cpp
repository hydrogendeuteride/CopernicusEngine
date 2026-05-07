#include "game/states/gameplay/prediction_kepler/kepler_prediction_builder.h"

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

        KeplerOrbitLineSet build_lines_for_arcs(const std::vector<KeplerOrbitArc> &arcs,
                                                const KeplerPredictionBuildRequest &request)
        {
            KeplerOrbitTessellationRequest line_request{};
            line_request.arcs = std::span<const KeplerOrbitArc>(arcs.data(), arcs.size());
            line_request.options = request.tessellation;
            line_request.body_state_provider = request.body_state_provider;
            line_request.world_frame = request.world_frame;
            return build_kepler_orbit_lines(line_request);
        }
    } // namespace

    KeplerPredictionBuildOutput build_kepler_prediction(const KeplerPredictionBuildRequest &request)
    {
        KeplerPredictionBuildOutput out{};
        out.maneuver_revision = request.maneuver_revision;

        if (!request.simulation || !finite_state(request.subject_state_inertial) ||
            !std::isfinite(request.t0_s))
        {
            out.status = KeplerOrbitStatus::InvalidInput;
            return out;
        }

        KeplerOrbitBuildRequest orbit_request{};
        orbit_request.simulation = request.simulation;
        orbit_request.ephemeris = request.ephemeris;
        orbit_request.subject_state_inertial = request.subject_state_inertial;
        orbit_request.t0_s = request.t0_s;
        orbit_request.requested_horizon_s = request.requested_horizon_s;
        orbit_request.fixed_primary_body_id = request.fixed_primary_body_id;
        orbit_request.current_primary_body_id = request.current_primary_body_id;
        orbit_request.options = request.options;

        out.orbit = build_kepler_orbit(orbit_request);
        if (!out.orbit.valid)
        {
            out.status = out.orbit.status;
            return out;
        }

        out.base_arcs.push_back(out.orbit.base_arc);
        out.metrics = compute_kepler_orbit_metrics(out.orbit.base_arc);

        out.base_lines = build_lines_for_arcs(out.base_arcs, request);
        if (!out.base_lines.valid)
        {
            out.status = out.base_lines.diagnostics.status;
            return out;
        }

        if (!request.maneuver_nodes.empty())
        {
            const KeplerManeuverSolveResult planned =
                    build_kepler_maneuver_arcs(out.orbit.base_arc,
                                               request.maneuver_nodes,
                                               request.options.propagation);
            if (!planned.valid)
            {
                out.status = planned.status;
                return out;
            }

            out.planned_arcs = planned.arcs;
            out.planned_lines = build_lines_for_arcs(out.planned_arcs, request);
            if (!out.planned_lines.valid)
            {
                out.status = out.planned_lines.diagnostics.status;
                return out;
            }
        }

        out.valid = true;
        out.status = KeplerOrbitStatus::Ok;
        return out;
    }
} // namespace Game
