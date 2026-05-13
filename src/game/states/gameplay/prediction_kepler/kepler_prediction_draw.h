#pragma once

#include "core/picking/line_pick_segment.h"
#include "core/orbit_plot/orbit_plot.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_state.h"

#include <glm/glm.hpp>

#include <vector>

class PickingSystem;

namespace Game
{
    // Runtime inputs for drawing already-built Kepler prediction line sets.
    // The prediction system owns geometry; this context supplies render targets,
    // picking hooks, visibility toggles, and view data needed by the draw pass.
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
        glm::vec4 planned_color{1.00f, 0.62f, 0.10f, 0.90f};
        OrbitPlotDepth depth{OrbitPlotDepth::DepthTested};
        float line_overlay_boost{0.35f};
        float planned_line_overlay_boost{0.0f};
        bool draw_planned_as_dashed{true};
        double dashed_segment_on_px{14.0};
        double dashed_segment_off_px{9.0};
        glm::mat4 viewproj{1.0f};
        WorldVec3 world_origin{0.0, 0.0, 0.0};
        double viewport_width_px{1280.0};
        double viewport_height_px{720.0};
    };

    // Emits visible Kepler tracks into OrbitPlotSystem and registers line picking
    // for maneuver creation on the active spacecraft track.
    void draw_kepler_prediction(const KeplerPredictionState &state,
                                const KeplerPredictionDrawContext &context);
} // namespace Game
