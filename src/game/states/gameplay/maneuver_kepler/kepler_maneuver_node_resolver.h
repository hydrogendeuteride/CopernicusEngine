#pragma once

#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_types.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_state.h"

#include <cstddef>
#include <vector>

namespace Game
{
    struct KeplerManeuverNodeResolveResult
    {
        std::size_t node_count{0};
        std::size_t valid_node_count{0};
        bool active_track_found{false};
        bool active_track_valid{false};
    };

    [[nodiscard]] const KeplerPredictionState::Track *find_active_player_kepler_track(
            const KeplerPredictionState &prediction);

    [[nodiscard]] KeplerManeuverNodeResolveResult resolve_kepler_maneuver_node_display_states(
            const KeplerManeuverPlanState &plan,
            const KeplerPredictionState &prediction,
            std::vector<KeplerManeuverNodeDisplayState> &out_states);

    [[nodiscard]] KeplerManeuverNodeResolveResult resolve_kepler_maneuver_node_display_states(
            const KeplerManeuverPlanState &plan,
            const KeplerPredictionState::Track *active_track,
            std::vector<KeplerManeuverNodeDisplayState> &out_states);
} // namespace Game
