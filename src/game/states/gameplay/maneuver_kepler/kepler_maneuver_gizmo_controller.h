#pragma once

#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_gizmo.h"

#include <glm/glm.hpp>

#include <span>
#include <vector>

namespace Game
{
    class KeplerManeuverGizmoController
    {
    public:
        // Camera-space pick ray used to map mouse motion onto a world-space maneuver axis.
        struct CameraRay
        {
            WorldVec3 camera_world{0.0, 0.0, 0.0};
            glm::dvec3 ray_origin_local{0.0, 0.0, 0.0};
            glm::dvec3 ray_dir_local{0.0, 0.0, -1.0};
        };

        // Result of one drag sample, including whether the caller should apply a new RTN delta-v.
        struct DragUpdateResult
        {
            bool sampled{false};
            bool clear_interaction{false};
            bool changed{false};
            orbitsim::Vec3 dv_rtn_mps{0.0, 0.0, 0.0};
        };

        // Starts an axis drag by capturing the initial RTN basis, delta-v, and ray-axis parameter.
        static bool begin_axis_drag(const KeplerManeuverPlanState &plan,
                                    std::span<const KeplerManeuverNodeDisplayState> display_states,
                                    KeplerManeuverInteraction &interaction, int node_id, KeplerManeuverHandleAxis axis,
                                    const CameraRay &ray, const glm::vec2 &mouse_pos_window);

        // Converts the current mouse ray into an updated RTN delta-v for the active axis drag.
        static DragUpdateResult update_axis_drag(KeplerManeuverInteraction &interaction,
                                                 const KeplerManeuverEditorNode &node,
                                                 const KeplerManeuverNodeDisplayState &display,
                                                 const glm::vec2 &mouse_pos_window, const CameraRay &ray,
                                                 float drag_threshold_px, double drag_sensitivity_mps_per_m,
                                                 bool ctrl_down, bool shift_down);

        // Builds projected hub, axis, and delete markers for the current maneuver overlay frame.
        static void build_markers(const KeplerManeuverPlanState &plan,
                                  std::span<const KeplerManeuverNodeDisplayState> display_states,
                                  const KeplerManeuverInteraction &interaction,
                                  const KeplerManeuverGizmoViewContext &view, float overlay_size_px,
                                  std::vector<KeplerManeuverHubMarker> &out_hubs,
                                  std::vector<KeplerManeuverAxisMarker> &out_axes,
                                  std::vector<KeplerManeuverDeleteMarker> &out_deletes);

    private:
        // Solves the ray-line closest-point normal equations and returns the line parameter.
        static bool closest_param_ray_line(const glm::dvec3 &ray_origin_local, const glm::dvec3 &ray_dir_local,
                                           const glm::dvec3 &line_origin_local, const glm::dvec3 &line_dir_local,
                                           double &out_line_t);
    };
} // namespace Game
