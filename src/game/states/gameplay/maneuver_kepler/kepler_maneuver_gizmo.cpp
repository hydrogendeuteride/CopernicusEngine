#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_gizmo.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Game::KeplerManeuverGizmo
{
    namespace
    {
        constexpr uint32_t rgba_u32(const uint32_t r, const uint32_t g, const uint32_t b, const uint32_t a)
        {
            return (a << 24u) | (b << 16u) | (g << 8u) | r;
        }

        double safe_length(const glm::dvec3 &v)
        {
            const double len2 = glm::dot(v, v);
            if (!(len2 > 0.0) || !std::isfinite(len2))
            {
                return 0.0;
            }
            return std::sqrt(len2);
        }
    } // namespace

    const char *axis_label(const KeplerManeuverHandleAxis axis)
    {
        switch (axis)
        {
            case KeplerManeuverHandleAxis::TangentialPos:
                return "+T";
            case KeplerManeuverHandleAxis::TangentialNeg:
                return "-T";
            case KeplerManeuverHandleAxis::RadialPos:
                return "+R";
            case KeplerManeuverHandleAxis::RadialNeg:
                return "-R";
            case KeplerManeuverHandleAxis::NormalPos:
                return "+N";
            case KeplerManeuverHandleAxis::NormalNeg:
                return "-N";
            default:
                return "";
        }
    }

    uint32_t axis_color(const KeplerManeuverHandleAxis axis)
    {
        switch (axis)
        {
            case KeplerManeuverHandleAxis::TangentialPos:
            case KeplerManeuverHandleAxis::TangentialNeg:
                return rgba_u32(95u, 235u, 105u, 220u);
            case KeplerManeuverHandleAxis::RadialPos:
            case KeplerManeuverHandleAxis::RadialNeg:
                return rgba_u32(240u, 100u, 100u, 220u);
            case KeplerManeuverHandleAxis::NormalPos:
            case KeplerManeuverHandleAxis::NormalNeg:
                return rgba_u32(95u, 160u, 255u, 220u);
            default:
                return rgba_u32(170u, 170u, 170u, 180u);
        }
    }

    bool is_axis(const KeplerManeuverHandleAxis axis)
    {
        return axis == KeplerManeuverHandleAxis::TangentialPos || axis == KeplerManeuverHandleAxis::TangentialNeg ||
               axis == KeplerManeuverHandleAxis::RadialPos || axis == KeplerManeuverHandleAxis::RadialNeg ||
               axis == KeplerManeuverHandleAxis::NormalPos || axis == KeplerManeuverHandleAxis::NormalNeg;
    }

    bool resolve_axis(const KeplerManeuverHandleAxis axis, const orbitsim::Vec3 &basis_r_world,
                      const orbitsim::Vec3 &basis_t_world, const orbitsim::Vec3 &basis_n_world,
                      orbitsim::Vec3 &out_axis_dir_world, int &out_component, double &out_sign)
    {
        out_axis_dir_world = orbitsim::Vec3{0.0, 1.0, 0.0};
        out_component = 1;
        out_sign = 1.0;

        switch (axis)
        {
            case KeplerManeuverHandleAxis::TangentialPos:
                out_axis_dir_world = basis_t_world;
                out_component = 1;
                out_sign = 1.0;
                break;
            case KeplerManeuverHandleAxis::TangentialNeg:
                out_axis_dir_world = -basis_t_world;
                out_component = 1;
                out_sign = -1.0;
                break;
            case KeplerManeuverHandleAxis::RadialPos:
                out_axis_dir_world = basis_r_world;
                out_component = 0;
                out_sign = 1.0;
                break;
            case KeplerManeuverHandleAxis::RadialNeg:
                out_axis_dir_world = -basis_r_world;
                out_component = 0;
                out_sign = -1.0;
                break;
            case KeplerManeuverHandleAxis::NormalPos:
                out_axis_dir_world = basis_n_world;
                out_component = 2;
                out_sign = 1.0;
                break;
            case KeplerManeuverHandleAxis::NormalNeg:
                out_axis_dir_world = -basis_n_world;
                out_component = 2;
                out_sign = -1.0;
                break;
            default:
                return false;
        }

        const double len = safe_length(out_axis_dir_world);
        if (!(len > 0.0) || !std::isfinite(len))
        {
            return false;
        }
        out_axis_dir_world /= len;
        return true;
    }

    bool is_occluded(const KeplerManeuverGizmoViewContext &view, const WorldVec3 &point_world)
    {
        if (!view.depth_occluder_valid)
        {
            return false;
        }

        const glm::dvec3 center_to_cam = view.camera_world - glm::dvec3(view.depth_occluder_center);
        const double center_to_cam_len2 = glm::dot(center_to_cam, center_to_cam);
        const double radius2 = view.depth_occluder_radius * view.depth_occluder_radius;
        if (!std::isfinite(center_to_cam_len2) || center_to_cam_len2 <= radius2)
        {
            return false;
        }

        const glm::dvec3 cam_to_point = glm::dvec3(point_world) - view.camera_world;
        const double point_dist = safe_length(cam_to_point);
        if (!std::isfinite(point_dist) || point_dist <= 1.0e-3)
        {
            return false;
        }

        const glm::dvec3 ray_dir = cam_to_point / point_dist;
        const glm::dvec3 oc = view.camera_world - glm::dvec3(view.depth_occluder_center);
        const double b = glm::dot(oc, ray_dir);
        const double c = glm::dot(oc, oc) - radius2;
        const double disc = b * b - c;
        if (!std::isfinite(disc) || disc <= 0.0)
        {
            return false;
        }

        const double t_near = -b - std::sqrt(disc);
        const double eps = std::max(1.0, view.depth_occluder_radius * 1.0e-6);
        return std::isfinite(t_near) && t_near > eps && t_near < (point_dist - eps);
    }

    bool project_point(const KeplerManeuverGizmoViewContext &view, const WorldVec3 &point_world, glm::vec2 &out_screen,
                       double &out_depth_m)
    {
        out_screen = glm::vec2(0.0f, 0.0f);
        out_depth_m = 0.0;

        const glm::dvec3 rel_world = glm::dvec3(point_world) - view.camera_world;
        const glm::dvec3 cam_space = view.world_to_cam * rel_world;
        const double depth_m = -cam_space.z;
        if (!std::isfinite(depth_m) || depth_m <= 1.0e-4)
        {
            return false;
        }

        const double ndc_x = cam_space.x / (depth_m * view.aspect * view.tan_half_fov);
        const double ndc_y = cam_space.y / (depth_m * view.tan_half_fov);
        if (!std::isfinite(ndc_x) || !std::isfinite(ndc_y))
        {
            return false;
        }
        if (ndc_x < -1.2 || ndc_x > 1.2 || ndc_y < -1.2 || ndc_y > 1.2)
        {
            return false;
        }

        const double u = (ndc_x + 1.0) * 0.5;
        const double v = (1.0 - ndc_y) * 0.5;
        if (!std::isfinite(u) || !std::isfinite(v))
        {
            return false;
        }

        const double swap_x =
            static_cast<double>(view.letterbox_rect.x) + u * static_cast<double>(view.letterbox_rect.width);
        const double swap_y =
            static_cast<double>(view.letterbox_rect.y) + v * static_cast<double>(view.letterbox_rect.height);
        const double draw_x = swap_x * view.draw_from_swap_x;
        const double draw_y = swap_y * view.draw_from_swap_y;
        const double win_x = draw_x * view.window_from_draw_x;
        const double win_y = draw_y * view.window_from_draw_y;
        if (!std::isfinite(win_x) || !std::isfinite(win_y))
        {
            return false;
        }

        out_screen = glm::vec2(static_cast<float>(win_x), static_cast<float>(win_y));
        out_depth_m = depth_m;
        return true;
    }

    const KeplerManeuverNodeDisplayState *find_display_state(
        const std::span<const KeplerManeuverNodeDisplayState> display_states, const int node_id)
    {
        for (const KeplerManeuverNodeDisplayState &state : display_states)
        {
            if (state.node_id == node_id)
            {
                return &state;
            }
        }
        return nullptr;
    }

    const KeplerManeuverNodeDisplaySnapshot *find_display_snapshot(
        const std::span<const KeplerManeuverNodeDisplaySnapshot> snapshots, const int node_id)
    {
        for (const KeplerManeuverNodeDisplaySnapshot &snapshot : snapshots)
        {
            if (snapshot.node_id == node_id)
            {
                return &snapshot;
            }
        }
        return nullptr;
    }

    KeplerManeuverGizmoHover find_hover(const std::span<const KeplerManeuverHubMarker> hubs,
                                        const std::span<const KeplerManeuverAxisMarker> axes,
                                        const std::span<const KeplerManeuverDeleteMarker> deletes,
                                        const glm::vec2 &mouse_pos, const float hub_hit_px2, const float axis_hit_px2,
                                        const float delete_hit_px2)
    {
        KeplerManeuverGizmoHover hover{};

        float best_delete_d2 = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < deletes.size(); ++i)
        {
            const glm::vec2 d = deletes[i].screen - mouse_pos;
            const float d2 = glm::dot(d, d);
            if (d2 <= delete_hit_px2 && d2 < best_delete_d2)
            {
                best_delete_d2 = d2;
                hover.delete_index = static_cast<int>(i);
            }
        }
        if (hover.delete_index >= 0)
        {
            const KeplerManeuverDeleteMarker &marker = deletes[static_cast<std::size_t>(hover.delete_index)];
            hover.node_id = marker.node_id;
            hover.state = KeplerManeuverInteraction::State::HoverDelete;
            return hover;
        }

        float best_axis_d2 = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < axes.size(); ++i)
        {
            const glm::vec2 d = axes[i].handle_screen - mouse_pos;
            const float d2 = glm::dot(d, d);
            if (d2 <= axis_hit_px2 && d2 < best_axis_d2)
            {
                best_axis_d2 = d2;
                hover.axis_index = static_cast<int>(i);
            }
        }
        if (hover.axis_index >= 0)
        {
            const KeplerManeuverAxisMarker &marker = axes[static_cast<std::size_t>(hover.axis_index)];
            hover.node_id = marker.node_id;
            hover.axis = marker.axis;
            hover.state = KeplerManeuverInteraction::State::HoverAxis;
            return hover;
        }

        float best_hub_d2 = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < hubs.size(); ++i)
        {
            const glm::vec2 d = hubs[i].screen - mouse_pos;
            const float d2 = glm::dot(d, d);
            if (d2 <= hub_hit_px2 && d2 < best_hub_d2)
            {
                best_hub_d2 = d2;
                hover.hub_index = static_cast<int>(i);
            }
        }
        if (hover.hub_index >= 0)
        {
            const KeplerManeuverHubMarker &marker = hubs[static_cast<std::size_t>(hover.hub_index)];
            hover.node_id = marker.node_id;
            hover.axis = KeplerManeuverHandleAxis::Hub;
            hover.state = KeplerManeuverInteraction::State::HoverHub;
        }

        return hover;
    }
} // namespace Game::KeplerManeuverGizmo
