#pragma once

#include "game/states/gameplay/prediction_nbody/gameplay_prediction_context.h"

#include <functional>

namespace Game
{
    class ManeuverSystem;
    class PredictionSystem;

    struct GameplayPredictionAccess
    {
        std::function<GameplayPredictionContext()> build_context;
        PredictionSystem &prediction;
        ManeuverSystem &maneuver;
        std::function<double()> current_sim_time_s;
        std::function<void()> mark_prediction_dirty;
    };
} // namespace Game
