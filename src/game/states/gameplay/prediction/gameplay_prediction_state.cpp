#include "game/states/gameplay/prediction/gameplay_prediction_state.h"

#include "game/orbit/orbit_prediction_service.h"

#include <memory>

namespace Game
{
    GameplayPredictionState::GameplayPredictionState()
        : service(std::make_unique<OrbitPredictionService>())
    {
    }

    GameplayPredictionState::~GameplayPredictionState() = default;

    OrbitPredictionService &GameplayPredictionState::solver_service()
    {
        if (!service)
        {
            service = std::make_unique<OrbitPredictionService>();
        }
        return *service;
    }

    const OrbitPredictionService &GameplayPredictionState::solver_service() const
    {
        return *service;
    }
} // namespace Game
