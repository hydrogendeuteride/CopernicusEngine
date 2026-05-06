#pragma once

#include "game/states/gameplay/maneuver/gameplay_state_maneuver_types.h"
#include "game/states/gameplay/maneuver/maneuver_prediction_bridge.h"
#include "game/states/gameplay/prediction/gameplay_prediction_access.h"

#include <functional>
#include <vector>

struct ImDrawList;

namespace Game
{
    class PredictionSystem;
    struct GameStateContext;

    struct ManeuverUiControllerContext
    {
        GameStateContext &ctx;
        ManeuverSystem &maneuver;
        PredictionSystem &prediction;
        bool &show_nodes_panel;
        bool debug_draw_enabled{false};
        GameplayPredictionAccess prediction_access;
        ManeuverPredictionBridge::Context maneuver_prediction;
        std::function<ManeuverCommandResult(const ManeuverCommand &)> apply_maneuver_command;
        std::function<void(GameStateContext &, bool)> refresh_maneuver_node_runtime_cache;
        std::function<double()> current_sim_time_s;
    };

    class ManeuverUiController
    {
    public:
        using Context = ManeuverUiControllerContext;

        static void emit_node_debug_overlay(Context &context);
        static void open_nodes_panel_from_orbit_pick_release(Context &context);
        static void draw_nodes_panel(Context &context);
        static void draw_imgui_gizmo(Context &context);

    private:
        static bool build_gizmo_view_context(const Context &context,
                                             ManeuverGizmoViewContext &out_view);
        static bool begin_axis_drag(Context &context,
                                    int node_id,
                                    ManeuverHandleAxis axis);
        static void apply_axis_drag(Context &context,
                                    ManeuverNode &node,
                                    const glm::vec2 &mouse_pos_window);
        static void update_ui_config(Context &context);
        static void draw_gizmo_markers(const Context &context,
                                       ImDrawList *draw_list,
                                       const std::vector<ManeuverHubMarker> &hubs,
                                       const std::vector<ManeuverAxisMarker> &handles,
                                       int hovered_hub_idx,
                                       int hovered_handle_idx,
                                       float hub_hit_px,
                                       float axis_hit_px);
        static void draw_gizmo_hover_tooltip(const Context &context,
                                             const std::vector<ManeuverAxisMarker> &handles,
                                             int hovered_handle_idx);
    };
} // namespace Game
