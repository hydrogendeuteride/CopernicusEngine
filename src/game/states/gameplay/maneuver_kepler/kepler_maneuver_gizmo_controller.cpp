#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_gizmo_controller.h"

#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_math.h"

#include <algorithm>
#include <cmath>

namespace Game
{
    namespace
    {
    } // namespace

    bool KeplerManeuverGizmoController::closest_param_ray_line(const glm::dvec3 &ray_origin_local,
                                                               const glm::dvec3 &ray_dir_local,
                                                               const glm::dvec3 &line_origin_local,
                                                               const glm::dvec3 &line_dir_local, double &out_line_t)
    {
        out_line_t = 0.0;

        // Normal-equation terms for the closest point between the mouse ray and maneuver axis line.
        const double uu = glm::dot(ray_dir_local, ray_dir_local);
        const double vv = glm::dot(line_dir_local, line_dir_local);
        if (!(uu > 0.0) || !(vv > 0.0) || !std::isfinite(uu) || !std::isfinite(vv))
        {
            return false;
        }

        const glm::dvec3 w0 = line_origin_local - ray_origin_local;
        const double uv = glm::dot(ray_dir_local, line_dir_local);
        const double uw = glm::dot(ray_dir_local, w0);
        const double vw = glm::dot(line_dir_local, w0);
        const double denom = uu * vv - uv * uv;

        double t = 0.0;
        // Solve the 2x2 normal equations unless the ray and line are nearly parallel.
        if (std::abs(denom) > 1.0e-9 && std::isfinite(denom))
        {
            const double s = (uw * vv - vw * uv) / denom;
            t = (uw * uv - vw * uu) / denom;
            if (!std::isfinite(s) || !std::isfinite(t))
            {
                return false;
            }
            if (s < 0.0)
            {
                // If the closest ray point is behind the camera, project from the ray origin instead.
                t = -vw / vv;
            }
        }
        else
        {
            // Parallel fallback: project the ray origin onto the axis line.
            t = -vw / vv;
            if (!std::isfinite(t))
            {
                return false;
            }
        }

        out_line_t = t;
        return std::isfinite(out_line_t);
    }

    bool KeplerManeuverGizmoController::begin_axis_drag(
        const KeplerManeuverPlanState &plan, const std::span<const KeplerManeuverNodeDisplayState> display_states,
        KeplerManeuverInteraction &interaction, const int node_id, const KeplerManeuverHandleAxis axis,
        const CameraRay &ray, const glm::vec2 &mouse_pos_window)
    {
        if (!KeplerManeuverGizmo::is_axis(axis))
        {
            return false;
        }

        // The drag is only meaningful when both the editable node and its rendered state exist.
        const KeplerManeuverEditorNode *node = plan.find_node(node_id);
        const KeplerManeuverNodeDisplayState *display =
            KeplerManeuverGizmo::find_display_state(display_states, node_id);
        if (!node || !display || !display->valid)
        {
            return false;
        }

        orbitsim::Vec3 axis_dir_world{};
        int component = 0;
        double sign = 1.0;
        if (!KeplerManeuverGizmo::resolve_axis(axis, display->basis_r_world, display->basis_t_world,
                                               display->basis_n_world, axis_dir_world, component, sign))
        {
            return false;
        }

        double start_t = 0.0;
        const glm::dvec3 axis_origin_local = glm::dvec3(display->position_world - ray.camera_world);
        if (!closest_param_ray_line(ray.ray_origin_local, ray.ray_dir_local, axis_origin_local, axis_dir_world,
                                    start_t))
        {
            return false;
        }

        // Freeze drag-time state so later orbit recomputation does not move the user's reference axis.
        interaction = {};
        interaction.state = KeplerManeuverInteraction::State::DragAxis;
        interaction.node_id = node_id;
        interaction.axis = axis;
        interaction.start_dv_rtn_mps = node->dv_rtn_mps;
        interaction.start_axis_t_m = start_t;
        interaction.drag_basis_r_world = display->basis_r_world;
        interaction.drag_basis_t_world = display->basis_t_world;
        interaction.drag_basis_n_world = display->basis_n_world;
        interaction.drag_start_mouse_window_pos = mouse_pos_window;
        interaction.drag_last_sample_mouse_window_pos = mouse_pos_window;
        interaction.drag_display_snapshots.clear();
        interaction.drag_display_snapshots.reserve(display_states.size());
        for (const KeplerManeuverNodeDisplayState &candidate : display_states)
        {
            if (!candidate.valid)
            {
                continue;
            }

            interaction.drag_display_snapshots.push_back(KeplerManeuverNodeDisplaySnapshot{
                .node_id = candidate.node_id,
                .position_world = candidate.position_world,
                .basis_r_world = candidate.basis_r_world,
                .basis_t_world = candidate.basis_t_world,
                .basis_n_world = candidate.basis_n_world,
            });
        }
        return true;
    }

    KeplerManeuverGizmoController::DragUpdateResult KeplerManeuverGizmoController::update_axis_drag(
        KeplerManeuverInteraction &interaction, const KeplerManeuverEditorNode &node,
        const KeplerManeuverNodeDisplayState &display, const glm::vec2 &mouse_pos_window, const CameraRay &ray,
        const float drag_threshold_px, const double drag_sensitivity_mps_per_m, const bool ctrl_down,
        const bool shift_down)
    {
        DragUpdateResult result{};
        if (interaction.state != KeplerManeuverInteraction::State::DragAxis || interaction.node_id != node.id ||
            !display.valid)
        {
            result.clear_interaction = true;
            return result;
        }

        const glm::vec2 drag_from_start = mouse_pos_window - interaction.drag_start_mouse_window_pos;
        // Keep clicks from becoming tiny accidental burns until ImGui's drag threshold is crossed.
        if (!interaction.drag_threshold_passed)
        {
            if (glm::dot(drag_from_start, drag_from_start) < (drag_threshold_px * drag_threshold_px))
            {
                return result;
            }
            interaction.drag_threshold_passed = true;
        }

        const glm::vec2 drag_since_last_sample = mouse_pos_window - interaction.drag_last_sample_mouse_window_pos;
        if (glm::dot(drag_since_last_sample, drag_since_last_sample) <= 1.0e-8f)
        {
            return result;
        }
        interaction.drag_last_sample_mouse_window_pos = mouse_pos_window;

        orbitsim::Vec3 axis_dir_world{};
        int component = 0;
        double sign = 1.0;
        if (!KeplerManeuverGizmo::resolve_axis(interaction.axis, interaction.drag_basis_r_world,
                                               interaction.drag_basis_t_world, interaction.drag_basis_n_world,
                                               axis_dir_world, component, sign))
        {
            result.clear_interaction = true;
            return result;
        }

        // Use the drag-start snapshot as the stable axis origin while the edited orbit updates.
        WorldVec3 axis_origin_world = display.position_world;
        if (const KeplerManeuverNodeDisplaySnapshot *drag_snapshot =
                KeplerManeuverGizmo::find_display_snapshot(interaction.drag_display_snapshots, node.id))
        {
            axis_origin_world = drag_snapshot->position_world;
        }

        double current_t = 0.0;
        const glm::dvec3 axis_origin_local = glm::dvec3(axis_origin_world - ray.camera_world);
        if (!closest_param_ray_line(ray.ray_origin_local, ray.ray_dir_local, axis_origin_local, axis_dir_world,
                                    current_t))
        {
            return result;
        }

        // Ctrl accelerates coarse edits, Shift slows down precision edits.
        double scale = std::max(0.00001, drag_sensitivity_mps_per_m);
        if (ctrl_down && !shift_down)
        {
            scale *= 10.0;
        }
        else if (shift_down && !ctrl_down)
        {
            scale *= 0.1;
        }

        // Apply the signed axis displacement to exactly one RTN component.
        const double delta_mps = (current_t - interaction.start_axis_t_m) * scale * sign;
        orbitsim::Vec3 next_dv = interaction.start_dv_rtn_mps;
        if (component == 0)
        {
            next_dv.x = interaction.start_dv_rtn_mps.x + delta_mps;
        }
        else if (component == 1)
        {
            next_dv.y = interaction.start_dv_rtn_mps.y + delta_mps;
        }
        else
        {
            next_dv.z = interaction.start_dv_rtn_mps.z + delta_mps;
        }

        result.sampled = true;
        if (kepler_finite_vec3(next_dv) &&
            KeplerManeuverMath::safe_length(next_dv - node.dv_rtn_mps) > 1.0e-7)
        {
            result.changed = true;
            result.dv_rtn_mps = next_dv;
        }
        return result;
    }

    void KeplerManeuverGizmoController::build_markers(
        const KeplerManeuverPlanState &plan, const std::span<const KeplerManeuverNodeDisplayState> display_states,
        const KeplerManeuverInteraction &interaction, const KeplerManeuverGizmoViewContext &view,
        const float overlay_size_px, std::vector<KeplerManeuverHubMarker> &out_hubs,
        std::vector<KeplerManeuverAxisMarker> &out_axes, std::vector<KeplerManeuverDeleteMarker> &out_deletes)
    {
        out_hubs.clear();
        out_axes.clear();
        out_deletes.clear();

        // Every visible node gets a hub marker, but only the selected node gets edit handles.
        out_hubs.reserve(display_states.size());
        for (const KeplerManeuverNodeDisplayState &display : display_states)
        {
            if (!display.valid || !plan.find_node(display.node_id))
            {
                continue;
            }
            if (KeplerManeuverGizmo::is_occluded(view, display.position_world))
            {
                continue;
            }

            KeplerManeuverHubMarker hub{};
            hub.node_id = display.node_id;
            if (!KeplerManeuverGizmo::project_point(view, display.position_world, hub.screen, hub.depth_m))
            {
                continue;
            }

            out_hubs.push_back(hub);
        }

        const int selected_node_id = plan.selected_node_id;
        const KeplerManeuverEditorNode *selected = plan.find_node(selected_node_id);
        const KeplerManeuverNodeDisplayState *selected_display =
            KeplerManeuverGizmo::find_display_state(display_states, selected_node_id);
        if (!selected || !selected_display || !selected_display->valid ||
            KeplerManeuverGizmo::is_occluded(view, selected_display->position_world))
        {
            return;
        }

        glm::vec2 hub_screen{};
        double hub_depth_m = 0.0;
        if (!KeplerManeuverGizmo::project_point(view, selected_display->position_world, hub_screen, hub_depth_m))
        {
            return;
        }

        // Convert a desired screen-space handle length into world meters at the selected node depth.
        double dist_m =
                KeplerManeuverMath::safe_length(glm::dvec3(selected_display->position_world) - view.camera_world);
        if (!std::isfinite(dist_m) || dist_m <= 1.0)
        {
            dist_m = std::max(1.0, hub_depth_m);
        }

        const double meters_per_px = (2.0 * view.tan_half_fov * dist_m) / view.logical_h;
        const float axis_len_px = std::max(24.0f, overlay_size_px * 1.9f);
        const double axis_offset_m = meters_per_px * static_cast<double>(axis_len_px);

        struct AxisDesc
        {
            KeplerManeuverHandleAxis axis{KeplerManeuverHandleAxis::None};
            orbitsim::Vec3 dir_world{0.0, 0.0, 0.0};
        };

        const AxisDesc axis_descs[6]{
            {KeplerManeuverHandleAxis::TangentialPos, selected_display->basis_t_world},
            {KeplerManeuverHandleAxis::TangentialNeg, -selected_display->basis_t_world},
            {KeplerManeuverHandleAxis::RadialPos, selected_display->basis_r_world},
            {KeplerManeuverHandleAxis::RadialNeg, -selected_display->basis_r_world},
            {KeplerManeuverHandleAxis::NormalPos, selected_display->basis_n_world},
            {KeplerManeuverHandleAxis::NormalNeg, -selected_display->basis_n_world},
        };

        out_axes.reserve(6);
        for (const AxisDesc &desc : axis_descs)
        {
            // Active drag handles are nudged outward to make the current axis visually clear.
            const orbitsim::Vec3 dir =
                    KeplerManeuverMath::normalized_or(desc.dir_world, orbitsim::Vec3{0.0, 1.0, 0.0});
            const bool active = interaction.state == KeplerManeuverInteraction::State::DragAxis &&
                                interaction.node_id == selected_node_id && interaction.axis == desc.axis;
            const double active_axis_offset_m = axis_offset_m * (active ? 1.14 : 1.0);
            const WorldVec3 handle_world = selected_display->position_world + dir * active_axis_offset_m;
            if (KeplerManeuverGizmo::is_occluded(view, handle_world))
            {
                continue;
            }

            KeplerManeuverAxisMarker marker{};
            marker.node_id = selected_node_id;
            marker.axis = desc.axis;
            marker.hub_screen = hub_screen;
            marker.base_color = KeplerManeuverGizmo::axis_color(desc.axis);
            marker.label = KeplerManeuverGizmo::axis_label(desc.axis);
            if (!KeplerManeuverGizmo::project_point(view, handle_world, marker.handle_screen, marker.depth_m))
            {
                continue;
            }
            out_axes.push_back(marker);
        }

        // Delete marker stays in screen space near the selected hub.
        KeplerManeuverDeleteMarker delete_marker{};
        delete_marker.node_id = selected_node_id;
        delete_marker.screen = hub_screen + glm::vec2(overlay_size_px * 1.45f, -overlay_size_px * 1.45f);
        delete_marker.depth_m = hub_depth_m;
        out_deletes.push_back(delete_marker);
    }
} // namespace Game
