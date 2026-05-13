#include "game/states/gameplay/prediction_kepler/kepler_prediction_builder.h"

#include <cmath>

namespace Game
{
    namespace
    {
        KeplerArcLineSet build_lines_for_arcs(const std::vector<KeplerOrbitArc> &arcs,
                                              const KeplerPredictionBuildRequest &request)
        {
            KeplerArcLineBuildRequest line_request{};
            line_request.arcs = std::span<const KeplerOrbitArc>(arcs.data(), arcs.size());
            line_request.options = request.line_options;
            line_request.body_state_provider = request.body_state_provider;
            line_request.world_frame = request.world_frame;
            return build_kepler_arc_lines(line_request);
        }

        void extend_last_planned_arc_to_preview_orbit(std::vector<KeplerOrbitArc> &arcs,
                                                      const KeplerPredictionOptions &options)
        {
            if (arcs.empty())
            {
                return;
            }

            KeplerOrbitArc &last_arc = arcs.back();
            const double preview_horizon_s = select_kepler_arc_horizon_s(last_arc.arc, options);
            if (!std::isfinite(preview_horizon_s) || preview_horizon_s <= 0.0)
            {
                return;
            }

            const double preview_end_s = last_arc.arc.t0_s + preview_horizon_s;
            if (std::isfinite(preview_end_s) && preview_end_s > last_arc.arc.t1_s)
            {
                last_arc.arc.t1_s = preview_end_s;
            }
        }
    } // namespace

    KeplerPredictionBuildOutput build_kepler_prediction(const KeplerPredictionBuildRequest &request)
    {
        KeplerPredictionBuildOutput out{};
        out.maneuver_revision = request.maneuver_revision;

        if (!request.simulation ||
            !kepler_finite_vec3(request.subject_state_inertial.position_m) ||
            !kepler_finite_vec3(request.subject_state_inertial.velocity_mps) ||
            !std::isfinite(request.t0_s))
        {
            out.status = KeplerOrbitStatus::InvalidInput;
            return out;
        }

        KeplerArcBuildRequest orbit_request{};
        orbit_request.simulation = request.simulation;
        orbit_request.ephemeris = request.ephemeris;
        orbit_request.subject_state_inertial = request.subject_state_inertial;
        orbit_request.t0_s = request.t0_s;
        orbit_request.requested_horizon_s = request.requested_horizon_s;
        orbit_request.fixed_primary_body_id = request.fixed_primary_body_id;
        orbit_request.current_primary_body_id = request.current_primary_body_id;
        orbit_request.options = request.options;

        out.orbit = build_kepler_arc(orbit_request);
        if (!out.orbit.valid)
        {
            out.status = out.orbit.status;
            return out;
        }

        out.base_arcs.push_back(out.orbit.base_arc);
        out.metrics = compute_kepler_arc_metrics(out.orbit.base_arc);

        out.base_lines = build_lines_for_arcs(out.base_arcs, request);
        if (!out.base_lines.valid)
        {
            out.status = out.base_lines.diagnostics.status;
            return out;
        }

        if (!request.maneuver_nodes.empty())
        {
            const KeplerManeuverArcBuildResult planned =
                    build_kepler_maneuver_arc_chain(out.orbit.base_arc,
                                                    request.maneuver_nodes,
                                                    request.options.propagation);
            if (planned.valid)
            {
                out.planned_arcs = planned.arcs;
                extend_last_planned_arc_to_preview_orbit(out.planned_arcs, request.options);
                out.planned_lines = build_lines_for_arcs(out.planned_arcs, request);
                if (!out.planned_lines.valid)
                {
                    out.planned_arcs.clear();
                    out.planned_lines = {};
                }
            }
        }

        out.valid = true;
        out.status = KeplerOrbitStatus::Ok;
        return out;
    }
} // namespace Game
