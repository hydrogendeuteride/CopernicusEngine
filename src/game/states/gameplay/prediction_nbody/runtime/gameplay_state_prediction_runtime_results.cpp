#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/prediction_nbody/gameplay_prediction_adapter.h"
#include "game/states/gameplay/prediction_nbody/prediction_host_context_builder.h"

#include <utility>

namespace Game
{
    void GameplayPredictionAdapter::apply_completed_prediction_derived_result(OrbitPredictionDerivedService::Result result)
    {
        (void) _access.prediction.apply_completed_derived_result(
                PredictionHostContextBuilder(context()).build(),
                std::move(result));
    }
} // namespace Game
