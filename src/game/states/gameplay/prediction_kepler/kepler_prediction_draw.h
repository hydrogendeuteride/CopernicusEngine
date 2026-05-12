#pragma once

#include "core/picking/line_pick_segment.h"
#include "core/orbit_plot/orbit_plot.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_state.h"

#include <glm/glm.hpp>

#include <vector>

class PickingSystem;

namespace Game
{
    struct KeplerPredictionDrawContext
    {
        OrbitPlotSystem *orbit_plot{nullptr};
        PickingSystem *picking{nullptr};
        std::vector<Picking::LinePickSegmentData> *pick_segment_scratch{nullptr};
        bool emit_pick{true};
        bool draw_planned{true};
        bool draw_orbiter_tracks{true};
        bool draw_celestial_kepler_tracks{false};
        bool draw_celestial_nbody_tracks{true};
        glm::vec4 base_color{0.18f, 0.82f, 1.00f, 0.78f};
        glm::vec4 planned_color{1.00f, 0.52f, 0.08f, 0.95f};
        OrbitPlotDepth depth{OrbitPlotDepth::DepthTested};
        float line_overlay_boost{0.35f};
        float planned_line_overlay_boost{0.55f};
        bool planned_dashed{true};
    };

    void draw_kepler_prediction(const KeplerPredictionState &state,
                                const KeplerPredictionDrawContext &context);
} // namespace Game
