#pragma once

#include "game/orbit/kepler/kepler_arc_builder.h"
#include "game/orbit/kepler/kepler_types.h"

#include <span>

namespace Game
{
    struct KeplerPatchChainBuildRequest
    {
        const orbitsim::GameSimulation *simulation{nullptr};
        const orbitsim::CelestialEphemeris *ephemeris{nullptr};
        KeplerBodyStateProvider body_state_provider{};
        orbitsim::State subject_state_inertial{};
        double t0_s{0.0};
        double t1_s{0.0};
        orbitsim::BodyId current_primary_body_id{orbitsim::kInvalidBodyId};
        orbitsim::BodyId fixed_initial_primary_body_id{orbitsim::kInvalidBodyId};
        KeplerPredictionOptions options{};
        std::span<const KeplerManeuverNode> maneuver_nodes{};
        bool extend_after_last_maneuver_to_preview_horizon{false};
    };

    KeplerPatchChainBuildResult build_kepler_patched_conics_chain(
            const KeplerPatchChainBuildRequest &request);
} // namespace Game
