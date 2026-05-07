#pragma once

#include "core/orbit_plot/orbit_plot.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_state.h"

#include <glm/glm.hpp>

class PickingSystem;

namespace Game
{
    struct KeplerPredictionDrawContext
    {
        OrbitPlotSystem *orbit_plot{nullptr};
        PickingSystem *picking{nullptr};
        bool emit_pick{true};
        bool draw_planned{true};
        bool draw_orbiter_tracks{true};
        bool draw_celestial_kepler_tracks{false};
        bool draw_celestial_nbody_tracks{true};
        glm::vec4 base_color{0.18f, 0.82f, 1.00f, 0.78f};
        glm::vec4 planned_color{0.35f, 1.00f, 0.50f, 0.90f};
        OrbitPlotDepth depth{OrbitPlotDepth::DepthTested};
    };

    void draw_kepler_prediction(const KeplerPredictionState &state,
                                const KeplerPredictionDrawContext &context);
} // namespace Game
