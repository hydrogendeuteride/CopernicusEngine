#pragma once

#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_commands.h"
#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_types.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <functional>
#include <span>
#include <vector>

namespace Game
{
    class KeplerManeuverSystem;
    struct GameStateContext;

    struct KeplerManeuverViewportRect
    {
        int32_t x{0};
        int32_t y{0};
        uint32_t width{0};
        uint32_t height{0};
    };

    struct KeplerManeuverGizmoViewContext
    {
        glm::dvec3 camera_world{0.0, 0.0, 0.0};
        glm::dmat3 world_to_cam{1.0};
        double logical_w{0.0};
        double logical_h{0.0};
        double aspect{1.0};
        double tan_half_fov{0.0};
        KeplerManeuverViewportRect letterbox_rect{};
        double draw_from_swap_x{1.0};
        double draw_from_swap_y{1.0};
        double window_from_draw_x{1.0};
        double window_from_draw_y{1.0};
        bool depth_occluder_valid{false};
        WorldVec3 depth_occluder_center{0.0, 0.0, 0.0};
        double depth_occluder_radius{0.0};
    };

    struct KeplerManeuverHubMarker
    {
        int node_id{-1};
        glm::vec2 screen{0.0f, 0.0f};
        double depth_m{0.0};
    };

    struct KeplerManeuverAxisMarker
    {
        int node_id{-1};
        KeplerManeuverHandleAxis axis{KeplerManeuverHandleAxis::None};
        glm::vec2 hub_screen{0.0f, 0.0f};
        glm::vec2 handle_screen{0.0f, 0.0f};
        uint32_t base_color{0u};
        const char *label{""};
        double depth_m{0.0};
    };

    struct KeplerManeuverDeleteMarker
    {
        int node_id{-1};
        glm::vec2 screen{0.0f, 0.0f};
        double depth_m{0.0};
    };

    struct KeplerManeuverGizmoHover
    {
        int hub_index{-1};
        int axis_index{-1};
        int delete_index{-1};
        int node_id{-1};
        KeplerManeuverHandleAxis axis{KeplerManeuverHandleAxis::None};
        KeplerManeuverInteraction::State state{KeplerManeuverInteraction::State::Idle};
    };

    struct KeplerManeuverGizmoDrawContext
    {
        GameStateContext &ctx;
        KeplerManeuverSystem &maneuver;
        KeplerManeuverGizmoViewContext view{};
        std::function<KeplerManeuverCommandResult(const KeplerManeuverCommand &)> apply_command;
    };

    namespace KeplerManeuverGizmo
    {
        const char *axis_label(KeplerManeuverHandleAxis axis);
        uint32_t axis_color(KeplerManeuverHandleAxis axis);
        bool is_axis(KeplerManeuverHandleAxis axis);
        bool resolve_axis(KeplerManeuverHandleAxis axis,
                          const orbitsim::Vec3 &basis_r_world,
                          const orbitsim::Vec3 &basis_t_world,
                          const orbitsim::Vec3 &basis_n_world,
                          orbitsim::Vec3 &out_axis_dir_world,
                          int &out_component,
                          double &out_sign);
        bool is_occluded(const KeplerManeuverGizmoViewContext &view, const WorldVec3 &point_world);
        bool project_point(const KeplerManeuverGizmoViewContext &view,
                           const WorldVec3 &point_world,
                           glm::vec2 &out_screen,
                           double &out_depth_m);
        const KeplerManeuverNodeDisplayState *find_display_state(
                std::span<const KeplerManeuverNodeDisplayState> display_states,
                int node_id);
        const KeplerManeuverNodeDisplaySnapshot *find_display_snapshot(
                std::span<const KeplerManeuverNodeDisplaySnapshot> snapshots,
                int node_id);
        KeplerManeuverGizmoHover find_hover(std::span<const KeplerManeuverHubMarker> hubs,
                                            std::span<const KeplerManeuverAxisMarker> axes,
                                            std::span<const KeplerManeuverDeleteMarker> deletes,
                                            const glm::vec2 &mouse_pos,
                                            float hub_hit_px2,
                                            float axis_hit_px2,
                                            float delete_hit_px2);
        void draw(KeplerManeuverGizmoDrawContext &context);
    } // namespace KeplerManeuverGizmo
} // namespace Game
