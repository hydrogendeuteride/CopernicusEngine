#pragma once

#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_types.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_state.h"

#include <cstddef>
#include <vector>

namespace Game
{
    // Summarizes whether maneuver node display resolution had usable prediction input.
    struct KeplerManeuverNodeResolveResult
    {
        std::size_t node_count{0};
        std::size_t valid_node_count{0};
        bool active_track_found{false};
        bool active_track_valid{false};
    };

    // Finds the prediction track currently owned by the active player.
    [[nodiscard]] const KeplerPredictionState::Track *find_active_player_kepler_track(
            const KeplerPredictionState &prediction);

    // Resolves authored maneuver nodes into world-space display data.
    [[nodiscard]] KeplerManeuverNodeResolveResult resolve_kepler_maneuver_node_display_states(
            const KeplerManeuverPlanState &plan,
            const KeplerPredictionState &prediction,
            std::vector<KeplerManeuverNodeDisplayState> &out_states);

    // Resolves authored maneuver nodes against a caller-selected prediction track.
    [[nodiscard]] KeplerManeuverNodeResolveResult resolve_kepler_maneuver_node_display_states(
            const KeplerManeuverPlanState &plan,
            const KeplerPredictionState::Track *active_track,
            std::vector<KeplerManeuverNodeDisplayState> &out_states);
} // namespace Game
