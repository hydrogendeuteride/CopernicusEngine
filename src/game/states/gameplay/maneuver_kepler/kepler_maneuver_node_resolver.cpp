#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_node_resolver.h"

#include "orbitsim/coordinate_frames.hpp"

#include <algorithm>
#include <cmath>

namespace Game
{
    namespace
    {
        constexpr double kTimeEpsilonS = 1.0e-9;

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

        bool times_equal(const double a, const double b)
        {
            return std::isfinite(a) && std::isfinite(b) && std::abs(a - b) <= kTimeEpsilonS;
        }

        bool time_in_arc_inclusive(const double t_s, const orbitsim::KeplerArc &arc)
        {
            if (!std::isfinite(t_s) || !orbitsim::kepler_arc_valid(arc))
            {
                return false;
            }

            const double t0_s = std::min(arc.t0_s, arc.t1_s);
            const double t1_s = std::max(arc.t0_s, arc.t1_s);
            return t_s + kTimeEpsilonS >= t0_s && t_s - kTimeEpsilonS <= t1_s;
        }

        double safe_length(const orbitsim::Vec3 &v)
        {
            const double len = glm::length(v);
            return std::isfinite(len) ? len : 0.0;
        }

        orbitsim::Vec3 normalized_or(const orbitsim::Vec3 &v, const orbitsim::Vec3 &fallback)
        {
            const double len = safe_length(v);
            if (len > 1.0e-12)
            {
                return v / len;
            }
            return fallback;
        }

        const KeplerOrbitArc *find_planned_pre_impulse_arc(
                const std::vector<KeplerOrbitArc> &arcs,
                const double node_time_s)
        {
            const KeplerOrbitArc *match = nullptr;
            for (const KeplerOrbitArc &arc : arcs)
            {
                if (orbitsim::kepler_arc_valid(arc.arc) && times_equal(arc.arc.t1_s, node_time_s))
                {
                    match = &arc;
                }
            }
            return match;
        }

        const KeplerOrbitArc *find_arc_containing_time(
                const std::vector<KeplerOrbitArc> &arcs,
                const double node_time_s)
        {
            for (const KeplerOrbitArc &arc : arcs)
            {
                if (time_in_arc_inclusive(node_time_s, arc.arc))
                {
                    return &arc;
                }
            }
            return nullptr;
        }

        const KeplerOrbitArc *select_base_arc(const KeplerPredictionState::Track &track,
                                              const double node_time_s)
        {
            if (const KeplerOrbitArc *arc = find_arc_containing_time(track.base_arcs, node_time_s))
            {
                return arc;
            }
            if (time_in_arc_inclusive(node_time_s, track.orbit.base_arc.arc))
            {
                return &track.orbit.base_arc;
            }
            return nullptr;
        }

        struct SelectedArc
        {
            const KeplerOrbitArc *arc{nullptr};
            KeplerManeuverNodeDisplaySource source{KeplerManeuverNodeDisplaySource::None};
        };

        SelectedArc select_node_arc(const KeplerPredictionState::Track &track,
                                    const double node_time_s)
        {
            if (!track.planned_arcs.empty())
            {
                if (const KeplerOrbitArc *arc =
                            find_planned_pre_impulse_arc(track.planned_arcs, node_time_s))
                {
                    return SelectedArc{
                            .arc = arc,
                            .source = KeplerManeuverNodeDisplaySource::PlannedPreImpulseArc,
                    };
                }
            }

            if (const KeplerOrbitArc *arc = select_base_arc(track, node_time_s))
            {
                return SelectedArc{
                        .arc = arc,
                        .source = KeplerManeuverNodeDisplaySource::BaseArc,
                };
            }

            return {};
        }

        bool resolve_body_state(const KeplerPredictionState::Track &track,
                                const orbitsim::BodyId body_id,
                                const double t_s,
                                const orbitsim::State &fallback,
                                orbitsim::State &out_state)
        {
            if (body_id != orbitsim::kInvalidBodyId &&
                track.body_state_provider.state_at &&
                track.body_state_provider.state_at(body_id, t_s, out_state) &&
                finite_state(out_state))
            {
                return true;
            }

            out_state = fallback;
            return finite_state(out_state);
        }

        KeplerManeuverNodeDisplayState make_unresolved_state(
                const KeplerManeuverPlanState &plan,
                const KeplerManeuverEditorNode &node,
                const KeplerManeuverNodeDisplayStatus status)
        {
            const double dv_mps = finite_vec3(node.dv_rtn_mps) ? safe_length(node.dv_rtn_mps) : 0.0;
            return KeplerManeuverNodeDisplayState{
                    .node_id = node.id,
                    .valid = false,
                    .selected = node.id == plan.selected_node_id,
                    .source = KeplerManeuverNodeDisplaySource::None,
                    .status = status,
                    .time_s = node.time_s,
                    .primary_body_id = node.primary_body_id,
                    .total_dv_mps = dv_mps,
            };
        }

        KeplerManeuverNodeDisplayState resolve_node_display_state(
                const KeplerManeuverPlanState &plan,
                const KeplerPredictionState::Track &track,
                const KeplerManeuverEditorNode &node)
        {
            if (!std::isfinite(node.time_s) || !finite_vec3(node.dv_rtn_mps))
            {
                return make_unresolved_state(plan,
                                             node,
                                             KeplerManeuverNodeDisplayStatus::InvalidNode);
            }

            const SelectedArc selected = select_node_arc(track, node.time_s);
            if (!selected.arc)
            {
                return make_unresolved_state(plan,
                                             node,
                                             KeplerManeuverNodeDisplayStatus::OutsideArc);
            }

            const orbitsim::KeplerArcSample sample =
                    orbitsim::sample_kepler_arc_state(selected.arc->arc,
                                                      node.time_s);
            if (!sample.ok() || !finite_state(sample.state_relative))
            {
                return make_unresolved_state(plan,
                                             node,
                                             KeplerManeuverNodeDisplayStatus::PropagationFailed);
            }

            orbitsim::State primary_state{};
            if (!resolve_body_state(track,
                                    selected.arc->arc.primary_body_id,
                                    sample.t_s,
                                    selected.arc->primary_state_inertial_at_t0,
                                    primary_state))
            {
                return make_unresolved_state(plan,
                                             node,
                                             KeplerManeuverNodeDisplayStatus::PrimaryUnavailable);
            }

            orbitsim::State reference_state{};
            if (!resolve_body_state(track,
                                    track.world_frame.world_reference_body_id,
                                    sample.t_s,
                                    track.world_frame.world_reference_state_inertial,
                                    reference_state))
            {
                return make_unresolved_state(plan,
                                             node,
                                             KeplerManeuverNodeDisplayStatus::PrimaryUnavailable);
            }

            const orbitsim::RtnFrame basis =
                    orbitsim::compute_rtn_frame(sample.state_relative.position_m,
                                                sample.state_relative.velocity_mps);
            const orbitsim::Vec3 basis_r = normalized_or(basis.R, {1.0, 0.0, 0.0});
            const orbitsim::Vec3 basis_t = normalized_or(basis.T, {0.0, 1.0, 0.0});
            const orbitsim::Vec3 basis_n = normalized_or(basis.N, {0.0, 0.0, 1.0});
            const orbitsim::Vec3 dv_world =
                    node.dv_rtn_mps.x * basis_r +
                    node.dv_rtn_mps.y * basis_t +
                    node.dv_rtn_mps.z * basis_n;

            const orbitsim::Vec3 inertial_position_m =
                    primary_state.position_m + sample.state_relative.position_m;
            const WorldVec3 position_world =
                    track.world_frame.world_reference_body_world +
                    WorldVec3(inertial_position_m - reference_state.position_m);

            return KeplerManeuverNodeDisplayState{
                    .node_id = node.id,
                    .valid = true,
                    .selected = node.id == plan.selected_node_id,
                    .source = selected.source,
                    .status = KeplerManeuverNodeDisplayStatus::Resolved,
                    .time_s = sample.t_s,
                    .primary_body_id = selected.arc->arc.primary_body_id,
                    .position_world = position_world,
                    .basis_r_world = basis_r,
                    .basis_t_world = basis_t,
                    .basis_n_world = basis_n,
                    .burn_direction_world = normalized_or(dv_world, basis_t),
                    .total_dv_mps = safe_length(node.dv_rtn_mps),
            };
        }
    } // namespace

    const KeplerPredictionState::Track *find_active_player_kepler_track(
            const KeplerPredictionState &prediction)
    {
        for (const KeplerPredictionState::Track &track : prediction.tracks)
        {
            if (track.active_player)
            {
                return &track;
            }
        }
        return nullptr;
    }

    KeplerManeuverNodeResolveResult resolve_kepler_maneuver_node_display_states(
            const KeplerManeuverPlanState &plan,
            const KeplerPredictionState &prediction,
            std::vector<KeplerManeuverNodeDisplayState> &out_states)
    {
        return resolve_kepler_maneuver_node_display_states(plan,
                                                           find_active_player_kepler_track(prediction),
                                                           out_states);
    }

    KeplerManeuverNodeResolveResult resolve_kepler_maneuver_node_display_states(
            const KeplerManeuverPlanState &plan,
            const KeplerPredictionState::Track *active_track,
            std::vector<KeplerManeuverNodeDisplayState> &out_states)
    {
        KeplerManeuverNodeResolveResult result{};
        result.node_count = plan.nodes.size();
        result.active_track_found = active_track != nullptr;
        result.active_track_valid = active_track && active_track->valid;

        out_states.clear();
        out_states.reserve(plan.nodes.size());

        if (!active_track)
        {
            for (const KeplerManeuverEditorNode &node : plan.nodes)
            {
                out_states.push_back(make_unresolved_state(
                        plan,
                        node,
                        KeplerManeuverNodeDisplayStatus::MissingActiveTrack));
            }
            return result;
        }

        if (!active_track->valid)
        {
            for (const KeplerManeuverEditorNode &node : plan.nodes)
            {
                out_states.push_back(make_unresolved_state(
                        plan,
                        node,
                        KeplerManeuverNodeDisplayStatus::InvalidTrack));
            }
            return result;
        }

        for (const KeplerManeuverEditorNode &node : plan.nodes)
        {
            KeplerManeuverNodeDisplayState display =
                    resolve_node_display_state(plan, *active_track, node);
            if (display.valid)
            {
                ++result.valid_node_count;
            }
            out_states.push_back(display);
        }

        return result;
    }
} // namespace Game
