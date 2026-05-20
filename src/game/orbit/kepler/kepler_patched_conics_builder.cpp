#include "game/orbit/kepler/kepler_patched_conics_builder.h"

#include "orbitsim/kepler_maneuver.hpp"
#include "orbitsim/soi.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace Game
{
    namespace
    {
        constexpr double kTimeEpsilonS = 1.0e-9;

        bool time_after(const double a, const double b)
        {
            return a > b + kTimeEpsilonS;
        }

        bool state_at_body(const KeplerPatchChainBuildRequest &request,
                           const orbitsim::BodyId body_id,
                           const double t_s,
                           orbitsim::State &out_state)
        {
            if (request.ephemeris && !request.ephemeris->empty())
            {
                std::size_t body_index = 0u;
                if (request.ephemeris->body_index_for_id(body_id, &body_index))
                {
                    out_state = request.ephemeris->body_state_at(body_index, t_s);
                    return kepler_finite_vec3(out_state.position_m) &&
                           kepler_finite_vec3(out_state.velocity_mps);
                }
            }

            if (request.body_state_provider.state_at &&
                request.body_state_provider.state_at(body_id, t_s, out_state))
            {
                return kepler_finite_vec3(out_state.position_m) &&
                       kepler_finite_vec3(out_state.velocity_mps);
            }

            if (!request.simulation)
            {
                return false;
            }
            const orbitsim::MassiveBody *body = request.simulation->body_by_id(body_id);
            if (!body)
            {
                return false;
            }

            out_state = body->state;
            return kepler_finite_vec3(out_state.position_m) &&
                   kepler_finite_vec3(out_state.velocity_mps);
        }

        KeplerPrimaryResolution resolve_primary_at_cursor(const KeplerPatchChainBuildRequest &request,
                                                          const orbitsim::State &subject_state_inertial,
                                                          const double t_s,
                                                          const orbitsim::BodyId current_primary_body_id,
                                                          const bool initial_primary)
        {
            KeplerPrimaryResolveRequest primary_request{};
            primary_request.simulation = request.simulation;
            primary_request.ephemeris = request.ephemeris;
            primary_request.query_position_m = subject_state_inertial.position_m;
            primary_request.query_time_s = t_s;
            primary_request.fixed_primary_body_id =
                    initial_primary ? request.fixed_initial_primary_body_id : current_primary_body_id;
            primary_request.current_primary_body_id = current_primary_body_id;
            primary_request.switch_options = request.options.primary_switch;
            primary_request.fallback_primary_hysteresis_keep_ratio =
                    request.options.fallback_primary_hysteresis_keep_ratio;
            return resolve_kepler_primary(primary_request);
        }

        std::optional<orbitsim::State> sample_arc_inertial_state(
                const KeplerPatchChainBuildRequest &request,
                const KeplerOrbitArc &arc,
                const double t_s,
                orbitsim::KeplerStatus &out_status)
        {
            const orbitsim::KeplerArcSample sample =
                    orbitsim::sample_kepler_arc_state(arc.arc,
                                                      t_s,
                                                      request.options.propagation);
            out_status = sample.diagnostics.status;
            if (!sample.ok())
            {
                return std::nullopt;
            }

            orbitsim::State primary_state{};
            if (!state_at_body(request, arc.arc.primary_body_id, t_s, primary_state))
            {
                out_status = orbitsim::KeplerStatus::InvalidFinalState;
                return std::nullopt;
            }

            orbitsim::State out_state = sample.state_relative;
            out_state.position_m += primary_state.position_m;
            out_state.velocity_mps += primary_state.velocity_mps;
            if (!kepler_finite_vec3(out_state.position_m) ||
                !kepler_finite_vec3(out_state.velocity_mps))
            {
                out_status = orbitsim::KeplerStatus::InvalidFinalState;
                return std::nullopt;
            }
            out_status = orbitsim::KeplerStatus::Ok;
            return out_state;
        }

        bool maneuver_node_valid(const KeplerManeuverNode &node)
        {
            return std::isfinite(node.t_s) && kepler_finite_vec3(node.dv_rtn_mps);
        }

        std::optional<std::size_t> next_maneuver_at_cursor(
                const std::span<const KeplerManeuverNode> nodes,
                const std::span<const uint8_t> consumed_nodes,
                const double cursor_t_s)
        {
            for (std::size_t i = 0; i < nodes.size(); ++i)
            {
                if (consumed_nodes[i] || !maneuver_node_valid(nodes[i]) ||
                    !kepler_same_sample_time(nodes[i].t_s, cursor_t_s))
                {
                    continue;
                }
                return i;
            }
            return std::nullopt;
        }

        std::optional<std::size_t> next_maneuver_after(
                const std::span<const KeplerManeuverNode> nodes,
                const std::span<const uint8_t> consumed_nodes,
                const double cursor_t_s,
                const double t_end_s)
        {
            std::optional<std::size_t> best{};
            for (std::size_t i = 0; i < nodes.size(); ++i)
            {
                const KeplerManeuverNode &node = nodes[i];
                if (consumed_nodes[i] ||
                    !maneuver_node_valid(node) ||
                    !time_after(node.t_s, cursor_t_s) ||
                    node.t_s > t_end_s + kTimeEpsilonS)
                {
                    continue;
                }
                if (!best.has_value() || node.t_s < nodes[*best].t_s)
                {
                    best = i;
                }
            }
            return best;
        }

        bool apply_maneuver_at_cursor(const KeplerPatchChainBuildRequest &request,
                                      const KeplerManeuverNode &node,
                                      const orbitsim::BodyId current_primary_body_id,
                                      orbitsim::State &cursor_state_inertial)
        {
            orbitsim::State primary_state{};
            if (!state_at_body(request, current_primary_body_id, node.t_s, primary_state))
            {
                return false;
            }

            orbitsim::State relative = cursor_state_inertial;
            relative.position_m -= primary_state.position_m;
            relative.velocity_mps -= primary_state.velocity_mps;
            const orbitsim::Vec3 dv_inertial =
                    orbitsim::rtn_delta_v_to_inertial(relative.position_m,
                                                      relative.velocity_mps,
                                                      node.dv_rtn_mps);
            if (!kepler_finite_vec3(dv_inertial))
            {
                return false;
            }

            cursor_state_inertial.velocity_mps += dv_inertial;
            return kepler_finite_vec3(cursor_state_inertial.velocity_mps);
        }

        double preview_end_time_for_arc(const KeplerPatchChainBuildRequest &request,
                                        const KeplerOrbitArc &arc,
                                        const KeplerPredictionOptions &options)
        {
            const double preview_horizon_s = select_kepler_base_arc_horizon_s(arc.arc, options);
            if (!std::isfinite(preview_horizon_s) || !(preview_horizon_s > 0.0))
            {
                return arc.arc.t1_s;
            }
            const double preview_end_s = arc.arc.t0_s + preview_horizon_s;
            if (!std::isfinite(preview_end_s))
            {
                return arc.arc.t1_s;
            }
            if (request.ephemeris && !request.ephemeris->empty())
            {
                const double ephemeris_end_s = request.ephemeris->t_end_s();
                if (std::isfinite(ephemeris_end_s) && ephemeris_end_s > arc.arc.t1_s)
                {
                    return std::min(preview_end_s, ephemeris_end_s);
                }
                return arc.arc.t1_s;
            }
            return preview_end_s;
        }
    } // namespace

    KeplerPatchChainBuildResult build_kepler_patched_conics_chain(
            const KeplerPatchChainBuildRequest &request)
    {
        KeplerPatchChainBuildResult out{};
        out.maneuver_diagnostics.candidate_impulses = request.maneuver_nodes.size();

        if (!request.simulation ||
            !kepler_finite_vec3(request.subject_state_inertial.position_m) ||
            !kepler_finite_vec3(request.subject_state_inertial.velocity_mps) ||
            !std::isfinite(request.t0_s) ||
            !std::isfinite(request.t1_s) ||
            !(request.t1_s > request.t0_s))
        {
            out.status = KeplerOrbitStatus::InvalidInput;
            return out;
        }

        const std::size_t max_patch_attempts =
                std::max<std::size_t>(1u, request.options.patched_conics.max_patches);
        const double min_patch_duration_s =
                kepler_positive_or_default(request.options.patched_conics.min_patch_duration_s, 1.0e-3);
        for (const KeplerManeuverNode &node : request.maneuver_nodes)
        {
            if (!maneuver_node_valid(node))
            {
                out.status = KeplerOrbitStatus::InvalidInput;
                return out;
            }
        }
        std::vector<uint8_t> consumed_maneuver_nodes(request.maneuver_nodes.size(), 0u);
        out.arcs.reserve(max_patch_attempts);
        out.events.reserve(max_patch_attempts + request.maneuver_nodes.size() + 1u);

        const orbitsim::CelestialEphemeris empty_ephemeris{};
        const orbitsim::CelestialEphemeris &ephemeris =
                request.ephemeris ? *request.ephemeris : empty_ephemeris;

        auto finalize = [&out](const KeplerOrbitStatus status) {
            out.maneuver_diagnostics.arcs_built = out.arcs.size();
            out.valid = !out.arcs.empty();
            out.status = status;
        };

        double cursor_t_s = request.t0_s;
        double chain_end_t_s = request.t1_s;
        orbitsim::State cursor_state_inertial = request.subject_state_inertial;
        orbitsim::BodyId current_primary_body_id = request.current_primary_body_id;
        bool initial_primary = true;
        std::size_t patch_attempts = 0u;

        while (cursor_t_s < chain_end_t_s - kTimeEpsilonS)
        {
            if (patch_attempts >= max_patch_attempts)
            {
                out.events.push_back(KeplerPatchEvent{
                        .t_s = cursor_t_s,
                        .from_primary_body_id = current_primary_body_id,
                        .to_primary_body_id = current_primary_body_id,
                        .reason = KeplerPatchBoundaryReason::PatchLimit,
                        .subject_state_inertial = cursor_state_inertial,
                });
                finalize(KeplerOrbitStatus::SampleBudgetExceeded);
                return out;
            }

            const KeplerPrimaryResolution primary =
                    resolve_primary_at_cursor(request,
                                              cursor_state_inertial,
                                              cursor_t_s,
                                              current_primary_body_id,
                                              initial_primary);
            initial_primary = false;
            if (!primary.valid)
            {
                out.status = primary.status;
                return out;
            }
            current_primary_body_id = primary.body_id;

            while (const std::optional<std::size_t> immediate_node_index =
                           next_maneuver_at_cursor(request.maneuver_nodes,
                                                   consumed_maneuver_nodes,
                                                   cursor_t_s))
            {
                const KeplerManeuverNode &node = request.maneuver_nodes[*immediate_node_index];
                if (node.primary_body_id != orbitsim::kInvalidBodyId &&
                    node.primary_body_id != current_primary_body_id)
                {
                    out.status = KeplerOrbitStatus::PrimaryMismatch;
                    return out;
                }
                out.events.push_back(KeplerPatchEvent{
                        .t_s = cursor_t_s,
                        .from_primary_body_id = current_primary_body_id,
                        .to_primary_body_id = current_primary_body_id,
                        .reason = KeplerPatchBoundaryReason::Maneuver,
                        .subject_state_inertial = cursor_state_inertial,
                });
                if (!apply_maneuver_at_cursor(request,
                                              node,
                                              current_primary_body_id,
                                              cursor_state_inertial))
                {
                    out.status = KeplerOrbitStatus::PropagationFailed;
                    out.maneuver_diagnostics.first_failure = orbitsim::KeplerStatus::InvalidFinalState;
                    return out;
                }
                consumed_maneuver_nodes[*immediate_node_index] = true;
                ++out.maneuver_diagnostics.impulses_applied;
            }
            ++patch_attempts;

            KeplerOrbitArc candidate{};
            candidate.arc.mu_m3_s2 = primary.mu_m3_s2;
            candidate.arc.primary_body_id = primary.body_id;
            candidate.arc.t0_s = cursor_t_s;
            candidate.arc.t1_s = chain_end_t_s;
            candidate.primary_state_inertial_at_t0 = primary.state_inertial;
            candidate.arc.state0_relative = cursor_state_inertial;
            candidate.arc.state0_relative.position_m -= primary.state_inertial.position_m;
            candidate.arc.state0_relative.velocity_mps -= primary.state_inertial.velocity_mps;
            if (!orbitsim::kepler_arc_valid(candidate.arc))
            {
                out.status = KeplerOrbitStatus::InvalidArc;
                return out;
            }

            const std::optional<std::size_t> next_node_index =
                    next_maneuver_after(request.maneuver_nodes,
                                        consumed_maneuver_nodes,
                                        cursor_t_s,
                                        chain_end_t_s);
            if (request.extend_after_last_maneuver_to_preview_horizon &&
                out.maneuver_diagnostics.impulses_applied > 0u &&
                !next_node_index.has_value())
            {
                const double preview_end_s = preview_end_time_for_arc(request, candidate, request.options);
                if (preview_end_s > chain_end_t_s)
                {
                    chain_end_t_s = preview_end_s;
                    candidate.arc.t1_s = chain_end_t_s;
                }
            }

            const double transition_limit_s =
                    next_node_index.has_value()
                            ? std::min(request.maneuver_nodes[*next_node_index].t_s, chain_end_t_s)
                            : chain_end_t_s;
            orbitsim::SoiTransitionSearchOptions transition_options{};
            transition_options.max_step_s = request.options.patched_conics.max_search_step_s;
            transition_options.refine_tolerance_s = request.options.patched_conics.refine_tolerance_s;
            transition_options.switch_options = request.options.primary_switch;
            transition_options.propagation = request.options.propagation;
            const orbitsim::SoiTransitionSearchResult transition =
                    orbitsim::find_next_soi_transition_on_kepler_arc(*request.simulation,
                                                                      ephemeris,
                                                                      candidate.arc,
                                                                      current_primary_body_id,
                                                                      transition_limit_s,
                                                                      transition_options);
            if (transition.first_failure != orbitsim::KeplerStatus::Ok)
            {
                out.status = KeplerOrbitStatus::PropagationFailed;
                return out;
            }

            double cut_t_s = chain_end_t_s;
            KeplerPatchBoundaryReason reason = KeplerPatchBoundaryReason::Horizon;
            orbitsim::BodyId next_primary_body_id = current_primary_body_id;
            bool terminal_budget_hit = false;

            if (transition.budget_hit)
            {
                cut_t_s = transition.last_tested_t_s;
                reason = KeplerPatchBoundaryReason::SearchBudget;
                terminal_budget_hit = true;
            }
            else if (transition.found &&
                transition.to_primary_body_id != orbitsim::kInvalidBodyId &&
                transition.t_s > cursor_t_s + kTimeEpsilonS &&
                transition.t_s < cut_t_s)
            {
                cut_t_s = transition.t_s;
                reason = KeplerPatchBoundaryReason::SoiTransition;
                next_primary_body_id = transition.to_primary_body_id;
            }

            if (next_node_index.has_value() &&
                request.maneuver_nodes[*next_node_index].t_s < cut_t_s + kTimeEpsilonS)
            {
                cut_t_s = request.maneuver_nodes[*next_node_index].t_s;
                reason = KeplerPatchBoundaryReason::Maneuver;
                next_primary_body_id = current_primary_body_id;
            }

            if (cut_t_s <= cursor_t_s + kTimeEpsilonS)
            {
                out.status = KeplerOrbitStatus::InvalidArc;
                return out;
            }

            candidate.arc.t1_s = cut_t_s;
            if (cut_t_s - cursor_t_s >= min_patch_duration_s ||
                reason == KeplerPatchBoundaryReason::Horizon)
            {
                out.arcs.push_back(candidate);
            }

            orbitsim::KeplerStatus sample_status = orbitsim::KeplerStatus::Ok;
            std::optional<orbitsim::State> cut_state =
                    sample_arc_inertial_state(request, candidate, cut_t_s, sample_status);
            if (!cut_state.has_value())
            {
                out.status = KeplerOrbitStatus::PropagationFailed;
                out.events.push_back(KeplerPatchEvent{
                        .t_s = cut_t_s,
                        .from_primary_body_id = current_primary_body_id,
                        .to_primary_body_id = current_primary_body_id,
                        .reason = KeplerPatchBoundaryReason::PropagationFailure,
                        .subject_state_inertial = cursor_state_inertial,
                });
                out.maneuver_diagnostics.first_failure = sample_status;
                return out;
            }

            cursor_state_inertial = *cut_state;
            out.events.push_back(KeplerPatchEvent{
                    .t_s = cut_t_s,
                    .from_primary_body_id = current_primary_body_id,
                    .to_primary_body_id = next_primary_body_id,
                    .reason = reason,
                    .subject_state_inertial = cursor_state_inertial,
            });

            if (reason == KeplerPatchBoundaryReason::Maneuver)
            {
                const KeplerManeuverNode &node = request.maneuver_nodes[*next_node_index];
                if (node.primary_body_id != orbitsim::kInvalidBodyId &&
                    node.primary_body_id != current_primary_body_id)
                {
                    out.status = KeplerOrbitStatus::PrimaryMismatch;
                    return out;
                }
                if (!apply_maneuver_at_cursor(request,
                                              node,
                                              current_primary_body_id,
                                              cursor_state_inertial))
                {
                    out.status = KeplerOrbitStatus::PropagationFailed;
                    out.maneuver_diagnostics.first_failure = orbitsim::KeplerStatus::InvalidFinalState;
                    return out;
                }
                consumed_maneuver_nodes[*next_node_index] = true;
                ++out.maneuver_diagnostics.impulses_applied;
            }
            else if (reason == KeplerPatchBoundaryReason::SoiTransition)
            {
                current_primary_body_id = next_primary_body_id;
            }

            if (terminal_budget_hit)
            {
                finalize(KeplerOrbitStatus::SampleBudgetExceeded);
                return out;
            }

            cursor_t_s = cut_t_s;
        }

        if (out.arcs.empty())
        {
            out.status = KeplerOrbitStatus::InvalidArc;
            return out;
        }

        finalize(KeplerOrbitStatus::Ok);
        return out;
    }
} // namespace Game
