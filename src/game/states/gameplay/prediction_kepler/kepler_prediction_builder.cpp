#include "game/states/gameplay/prediction_kepler/kepler_prediction_builder.h"

#include "game/orbit/kepler/kepler_patched_conics_builder.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>

namespace Game
{
    namespace
    {
        constexpr double kPostLastManeuverNodePadS = 1.0;
        using PredictionBuildPerfClock = std::chrono::steady_clock;

        double elapsed_ms(const PredictionBuildPerfClock::time_point start,
                          const PredictionBuildPerfClock::time_point end = PredictionBuildPerfClock::now())
        {
            return std::chrono::duration<double, std::milli>(end - start).count();
        }

        // Used to extend the base arc far enough for all burns.
        double latest_finite_maneuver_node_time_s(const std::span<const KeplerManeuverNode> nodes)
        {
            double latest_t_s = -std::numeric_limits<double>::infinity();
            for (const KeplerManeuverNode &node : nodes)
            {
                if (std::isfinite(node.t_s))
                {
                    latest_t_s = std::max(latest_t_s, node.t_s);
                }
            }
            return latest_t_s;
        }

        // Used to avoid drawing a duplicate pre-burn segment.
        double earliest_finite_maneuver_node_time_s(const std::span<const KeplerManeuverNode> nodes)
        {
            double earliest_t_s = std::numeric_limits<double>::infinity();
            for (const KeplerManeuverNode &node : nodes)
            {
                if (std::isfinite(node.t_s))
                {
                    earliest_t_s = std::min(earliest_t_s, node.t_s);
                }
            }
            return earliest_t_s;
        }

        KeplerOrbitArc extend_base_arc_to_cover_maneuver_nodes(
                KeplerOrbitArc base_arc,
                const std::span<const KeplerManeuverNode> nodes)
        {
            const double latest_t_s = latest_finite_maneuver_node_time_s(nodes);
            if (std::isfinite(latest_t_s) && latest_t_s >= base_arc.arc.t1_s)
            {
                base_arc.arc.t1_s = latest_t_s + kPostLastManeuverNodePadS;
            }
            return base_arc;
        }

        // Skips duplicate planned segments before the first burn.
        std::size_t find_first_planned_draw_arc_index(
                const std::vector<KeplerOrbitArc> &arcs,
                const std::span<const KeplerManeuverNode> nodes)
        {
            if (arcs.empty())
            {
                return 0u;
            }

            std::size_t first_visible_arc_index = 0u;
            const double first_node_time_s = earliest_finite_maneuver_node_time_s(nodes);
            if (arcs.size() > 1u && std::isfinite(first_node_time_s))
            {
                while (first_visible_arc_index + 1u < arcs.size())
                {
                    const KeplerOrbitArc &arc = arcs[first_visible_arc_index];
                    if (arc.arc.t1_s > first_node_time_s + 1.0e-9 ||
                        kepler_same_sample_time(arc.arc.t0_s, first_node_time_s))
                    {
                        break;
                    }
                    ++first_visible_arc_index;
                }
            }
            return first_visible_arc_index;
        }

        // Keeps the post-burn orbit visible after the last node.
        void extend_last_planned_arc_to_preview_orbit(std::vector<KeplerOrbitArc> &arcs,
                                                      const KeplerPredictionOptions &options)
        {
            if (arcs.empty())
            {
                return;
            }

            KeplerOrbitArc &last_arc = arcs.back();
            const double preview_horizon_s = select_kepler_base_arc_horizon_s(last_arc.arc, options);
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

        KeplerPatchChainBuildRequest make_patch_chain_request(
                const KeplerPredictionBuildRequest &request,
                const KeplerBaseArcBuildResult &orbit,
                const double t1_s,
                const std::span<const KeplerManeuverNode> maneuver_nodes,
                const bool extend_after_last_maneuver_to_preview_horizon)
        {
            KeplerPatchChainBuildRequest chain_request{};
            chain_request.simulation = request.simulation;
            chain_request.ephemeris = request.ephemeris;
            chain_request.body_state_provider = request.body_state_provider;
            chain_request.subject_state_inertial = request.subject_state_inertial;
            chain_request.t0_s = request.t0_s;
            chain_request.t1_s = t1_s;
            chain_request.current_primary_body_id = orbit.primary.body_id;
            chain_request.fixed_initial_primary_body_id = orbit.primary.body_id;
            chain_request.options = request.options;
            chain_request.maneuver_nodes = maneuver_nodes;
            chain_request.extend_after_last_maneuver_to_preview_horizon =
                    extend_after_last_maneuver_to_preview_horizon;
            return chain_request;
        }
    } // namespace

    double required_kepler_maneuver_node_horizon_s(const double t0_s,
                                                  const KeplerManeuverNode *nodes,
                                                  const std::size_t node_count)
    {
        if (!std::isfinite(t0_s) || !nodes || node_count == 0u)
        {
            return 0.0;
        }

        double latest_future_node_t_s = -std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < node_count; ++i)
        {
            const double node_t_s = nodes[i].t_s;
            if (std::isfinite(node_t_s) && node_t_s >= t0_s)
            {
                latest_future_node_t_s = std::max(latest_future_node_t_s, node_t_s);
            }
        }

        return std::isfinite(latest_future_node_t_s)
                       ? (latest_future_node_t_s - t0_s) + kPostLastManeuverNodePadS
                       : 0.0;
    }

    double required_kepler_planned_preview_horizon_s(const KeplerBaseArcBuildResult &orbit,
                                                     const KeplerManeuverNode *nodes,
                                                     const std::size_t node_count,
                                                     const KeplerPredictionOptions &options)
    {
        if (!orbit.valid ||
            !std::isfinite(orbit.base_arc.arc.t0_s) ||
            !nodes ||
            node_count == 0u)
        {
            return 0.0;
        }

        const double t0_s = orbit.base_arc.arc.t0_s;
        double latest_future_node_t_s = -std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < node_count; ++i)
        {
            const double node_t_s = nodes[i].t_s;
            if (std::isfinite(node_t_s) && node_t_s >= t0_s)
            {
                latest_future_node_t_s = std::max(latest_future_node_t_s, node_t_s);
            }
        }
        if (!std::isfinite(latest_future_node_t_s))
        {
            return 0.0;
        }

        const double fallback_preview_horizon_s =
                kepler_positive_or_default(options.open_orbit_window_s,
                                            24.0 * 60.0 * 60.0);
        const double fallback_horizon_s = (latest_future_node_t_s - t0_s) +
                                          fallback_preview_horizon_s;

        KeplerOrbitArc maneuver_base_arc = orbit.base_arc;
        if (latest_future_node_t_s >= maneuver_base_arc.arc.t1_s)
        {
            maneuver_base_arc.arc.t1_s = latest_future_node_t_s + kPostLastManeuverNodePadS;
        }

        const KeplerManeuverArcBuildResult planned =
                build_kepler_maneuver_arc_chain(maneuver_base_arc,
                                                std::span<const KeplerManeuverNode>(nodes, node_count),
                                                options.propagation);
        if (!planned.valid || planned.arcs.empty())
        {
            return fallback_horizon_s;
        }

        const KeplerOrbitArc &last_arc = planned.arcs.back();
        const double preview_horizon_s = select_kepler_base_arc_horizon_s(last_arc.arc, options);
        const double preview_end_s = last_arc.arc.t0_s + preview_horizon_s;
        if (!std::isfinite(preview_horizon_s) ||
            !(preview_horizon_s > 0.0) ||
            !std::isfinite(preview_end_s) ||
            !(preview_end_s > t0_s))
        {
            return fallback_horizon_s;
        }

        return preview_end_s - t0_s;
    }

    double required_kepler_planned_preview_ephemeris_horizon_s(
            const std::span<const KeplerOrbitArc> planned_arcs,
            const std::span<const KeplerPatchEvent> planned_events,
            const double t0_s,
            const double ephemeris_end_s,
            const KeplerPredictionOptions &options)
    {
        if (planned_arcs.empty() ||
            planned_events.empty() ||
            !std::isfinite(t0_s) ||
            !std::isfinite(ephemeris_end_s))
        {
            return 0.0;
        }

        const KeplerOrbitArc &last_arc = planned_arcs.back();
        const KeplerPatchEvent &last_event = planned_events.back();
        if (last_event.reason != KeplerPatchBoundaryReason::Horizon ||
            !kepler_same_sample_time(last_event.t_s, last_arc.arc.t1_s) ||
            last_arc.arc.t1_s + 1.0e-6 < ephemeris_end_s)
        {
            return 0.0;
        }

        const double preview_horizon_s = select_kepler_base_arc_horizon_s(last_arc.arc, options);
        const double preview_end_s = last_arc.arc.t0_s + preview_horizon_s;
        if (!std::isfinite(preview_horizon_s) ||
            !(preview_horizon_s > 0.0) ||
            !std::isfinite(preview_end_s) ||
            preview_end_s <= ephemeris_end_s + 1.0e-6)
        {
            return 0.0;
        }

        const double required_horizon_s = preview_end_s - t0_s;
        return (std::isfinite(required_horizon_s) && required_horizon_s > 0.0)
                       ? required_horizon_s
                       : 0.0;
    }

    // Builds one subject's base and optional maneuver prediction.
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

        KeplerBaseArcBuildRequest orbit_request{};
        orbit_request.simulation = request.simulation;
        orbit_request.ephemeris = request.ephemeris;
        orbit_request.subject_state_inertial = request.subject_state_inertial;
        orbit_request.t0_s = request.t0_s;
        orbit_request.requested_horizon_s = request.requested_horizon_s;
        orbit_request.fixed_primary_body_id = request.fixed_primary_body_id;
        orbit_request.current_primary_body_id = request.current_primary_body_id;
        orbit_request.options = request.options;

        // Everything else depends on a valid base orbit.
        const auto orbit_start = PredictionBuildPerfClock::now();
        out.orbit = build_kepler_base_arc(orbit_request);
        out.perf.orbit_ms = elapsed_ms(orbit_start);
        if (!out.orbit.valid)
        {
            out.status = out.orbit.status;
            return out;
        }

        if (request.options.patched_conics.enabled)
        {
            const auto base_patch_start = PredictionBuildPerfClock::now();
            const KeplerPatchChainBuildResult base_chain =
                    build_kepler_patched_conics_chain(
                            make_patch_chain_request(request,
                                                     out.orbit,
                                                      out.orbit.base_arc.arc.t1_s,
                                                      {},
                                                      false));
            out.perf.base_patch_ms = elapsed_ms(base_patch_start);
            if (!base_chain.valid)
            {
                out.status = base_chain.status;
                return out;
            }
            out.base_arcs = base_chain.arcs;
            out.base_patch_events = base_chain.events;
            out.status = base_chain.status;
        }
        else
        {
            out.base_arcs.push_back(out.orbit.base_arc);
            out.status = KeplerOrbitStatus::Ok;
        }
        out.metrics = compute_kepler_arc_metrics(out.orbit.base_arc);

        if (!request.maneuver_nodes.empty())
        {
            out.planned_requested = true;

            if (request.options.patched_conics.enabled)
            {
                const auto planned_arc_start = PredictionBuildPerfClock::now();
                const double latest_node_t_s = latest_finite_maneuver_node_time_s(request.maneuver_nodes);
                const double planned_t1_s =
                        std::isfinite(latest_node_t_s)
                                ? std::max(out.orbit.base_arc.arc.t1_s,
                                           latest_node_t_s + kPostLastManeuverNodePadS)
                                : out.orbit.base_arc.arc.t1_s;
                const KeplerPatchChainBuildResult planned =
                        build_kepler_patched_conics_chain(
                                make_patch_chain_request(request,
                                                         out.orbit,
                                                         planned_t1_s,
                                                          request.maneuver_nodes,
                                                          true));
                out.perf.planned_arc_ms = elapsed_ms(planned_arc_start);
                out.planned_status = planned.status;
                out.planned_diagnostics = planned.maneuver_diagnostics;
                if (planned.valid)
                {
                    out.planned_arcs = planned.arcs;
                    out.planned_patch_events = planned.events;
                }
            }
            else
            {
                const auto planned_arc_start = PredictionBuildPerfClock::now();
                // Turn normalized maneuver nodes into planned arcs.
                const KeplerOrbitArc maneuver_base_arc =
                        extend_base_arc_to_cover_maneuver_nodes(out.orbit.base_arc,
                                                                request.maneuver_nodes);
                const KeplerManeuverArcBuildResult planned =
                        build_kepler_maneuver_arc_chain(maneuver_base_arc,
                                                        request.maneuver_nodes,
                                                        request.options.propagation);
                out.planned_status = planned.status;
                out.planned_diagnostics = planned.diagnostics;
                if (planned.valid)
                {
                    out.planned_arcs = planned.arcs;
                    extend_last_planned_arc_to_preview_orbit(out.planned_arcs, request.options);
                }
                out.perf.planned_arc_ms = elapsed_ms(planned_arc_start);
            }

            if (!out.planned_arcs.empty())
            {
                out.first_planned_draw_arc_index =
                        find_first_planned_draw_arc_index(out.planned_arcs, request.maneuver_nodes);
                out.planned_valid = true;
                if (out.planned_status == KeplerOrbitStatus::Ok)
                {
                    out.planned_status = KeplerOrbitStatus::Ok;
                }
            }
        }

        out.valid = true;
        if (out.status == KeplerOrbitStatus::InvalidInput)
        {
            out.status = KeplerOrbitStatus::Ok;
        }
        return out;
    }
} // namespace Game
