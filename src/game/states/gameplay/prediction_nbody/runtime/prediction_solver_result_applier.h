#pragma once

#include "game/states/gameplay/prediction_nbody/gameplay_prediction_derived_service.h"
#include "game/states/gameplay/prediction_nbody/runtime/prediction_runtime_context.h"

#include <optional>

namespace Game
{
    struct PredictionSolverResultApplyResult
    {
        std::optional<OrbitPredictionDerivedService::Request> derived_request{};
    };

    class PredictionSolverResultApplier
    {
    public:
        static PredictionSolverResultApplyResult apply_solver_result(
                PredictionTrackState &track,
                OrbitPredictionResult result,
                const PredictionRuntimeContext &context);
    };
} // namespace Game
