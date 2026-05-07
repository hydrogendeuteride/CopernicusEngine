#include "game/orbit/kepler/kepler_maneuver_solver.h"

#include "orbitsim/kepler_maneuver.hpp"

#include <cmath>
#include <vector>

namespace Game
{
    namespace
    {
        bool finite_vec3(const orbitsim::Vec3 &v)
        {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        }

        bool time_in_arc_half_open(const double t_s, const double t0_s, const double t1_s)
        {
            if (t1_s >= t0_s)
            {
                return t_s >= t0_s && t_s < t1_s;
            }
            return t_s <= t0_s && t_s > t1_s;
        }
    } // namespace

    KeplerManeuverSolveResult build_kepler_maneuver_arcs(
            const KeplerOrbitArc &base_arc,
            const std::span<const KeplerManeuverNode> nodes,
            const orbitsim::KeplerPropagationOptions &propagation_options)
    {
        KeplerManeuverSolveResult out{};
        if (!orbitsim::kepler_arc_valid(base_arc.arc))
        {
            out.status = KeplerOrbitStatus::InvalidArc;
            return out;
        }

        std::vector<orbitsim::KeplerImpulse> impulses;
        impulses.reserve(nodes.size());
        for (const KeplerManeuverNode &node : nodes)
        {
            if (!std::isfinite(node.t_s) || !finite_vec3(node.dv_rtn_mps))
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
