#pragma once

#include "game/orbit/kepler/kepler_arc_builder.h"
#include "game/orbit/kepler/kepler_arc_info.h"
#include "game/orbit/kepler/kepler_arc_line_builder.h"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace Game
{
    // Inputs for one Kepler prediction track.
    struct KeplerPredictionBuildRequest
    {
        const orbitsim::GameSimulation *simulation{nullptr};
        const orbitsim::CelestialEphemeris *ephemeris{nullptr};
        orbitsim::State subject_state_inertial{};
        KeplerWorldFrame world_frame{};
        KeplerBodyStateProvider body_state_provider{};
        double t0_s{0.0};
        double requested_horizon_s{0.0};
        orbitsim::BodyId fixed_primary_body_id{orbitsim::kInvalidBodyId};
        orbitsim::BodyId current_primary_body_id{orbitsim::kInvalidBodyId};
        KeplerPredictionOptions options{};
        KeplerArcLineOptions line_options{};
        std::span<const KeplerManeuverNode> maneuver_nodes{};
        uint64_t maneuver_revision{0};
    };

    // Built arcs and render lines for one subject.
    struct KeplerPredictionBuildOutput
    {
        bool valid{false};
        KeplerOrbitStatus status{KeplerOrbitStatus::InvalidInput};
        uint64_t maneuver_revision{0};
        KeplerArcBuildResult orbit{};
        std::vector<KeplerOrbitArc> base_arcs{};
        std::vector<KeplerOrbitArc> planned_arcs{};
        std::vector<KeplerPatchEvent> base_patch_events{};
        std::vector<KeplerPatchEvent> planned_patch_events{};
        KeplerArcLineSet base_lines{};
        KeplerArcLineSet planned_lines{};
        bool planned_requested{false};
        bool planned_valid{false};
        KeplerOrbitStatus planned_status{KeplerOrbitStatus::Ok};
        orbitsim::KeplerManeuverDiagnostics planned_diagnostics{};
        KeplerArcLineDiagnostics planned_line_diagnostics{};
        KeplerArcMetrics metrics{};
    };

    // Builds analytic arcs and their renderable line vertices.
    KeplerPredictionBuildOutput build_kepler_prediction(
            const KeplerPredictionBuildRequest &request);

    // Returns the future horizon needed to cover authored maneuver nodes.
    double required_kepler_maneuver_node_horizon_s(
            double t0_s,
            const KeplerManeuverNode *nodes,
            std::size_t node_count);

    // Returns the horizon needed for the post-burn preview segment.
    double required_kepler_planned_preview_horizon_s(
            const KeplerArcBuildResult &orbit,
            const KeplerManeuverNode *nodes,
            std::size_t node_count,
            const KeplerPredictionOptions &options);
} // namespace Game
