#pragma once

#include "game/orbit/kepler/kepler_arc_builder.h"
#include "game/orbit/kepler/kepler_arc_info.h"
#include "game/orbit/kepler/kepler_arc_line_builder.h"

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
        KeplerArcLineSet base_lines{};
        KeplerArcLineSet planned_lines{};
        KeplerArcMetrics metrics{};
    };

    // Builds analytic arcs and their renderable line vertices.
    KeplerPredictionBuildOutput build_kepler_prediction(
            const KeplerPredictionBuildRequest &request);
} // namespace Game
