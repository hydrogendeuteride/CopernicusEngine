#pragma once

#include "game/orbit/kepler/kepler_maneuver_solver.h"
#include "game/orbit/kepler/kepler_orbit_builder.h"
#include "game/orbit/kepler/kepler_orbit_metrics.h"
#include "game/orbit/kepler/kepler_orbit_tessellator.h"

#include <cstdint>
#include <span>
#include <vector>

namespace Game
{
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
        KeplerOrbitTessellationOptions tessellation{};
        std::span<const KeplerManeuverNode> maneuver_nodes{};
        uint64_t maneuver_revision{0};
    };

    struct KeplerPredictionBuildOutput
    {
        bool valid{false};
        KeplerOrbitStatus status{KeplerOrbitStatus::InvalidInput};
        uint64_t maneuver_revision{0};
        KeplerOrbitBuildResult orbit{};
        std::vector<KeplerOrbitArc> base_arcs{};
        std::vector<KeplerOrbitArc> planned_arcs{};
        KeplerOrbitLineSet base_lines{};
        KeplerOrbitLineSet planned_lines{};
        KeplerOrbitMetrics metrics{};
    };

    [[nodiscard]] KeplerPredictionBuildOutput build_kepler_prediction(
            const KeplerPredictionBuildRequest &request);
} // namespace Game
