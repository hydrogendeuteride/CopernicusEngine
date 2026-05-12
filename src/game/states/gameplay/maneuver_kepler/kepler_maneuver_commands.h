#pragma once

#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_types.h"

#include <vector>

namespace Game
{
    enum class KeplerManeuverCommandKind
    {
        None = 0,
        AddNode,
        ClearPlan,
        SelectNode,
        SelectNodeByIndex,
        EnsureSelection,
        RemoveNode,
        PrunePastNodes,
        SetNodeTime,
        SetNodeDv,
        SetNodePrimaryBody,
        SortByTime,
        MarkPlanDirty,
    };

    struct KeplerManeuverCommand
    {
        KeplerManeuverCommandKind kind{KeplerManeuverCommandKind::None};
        KeplerManeuverEditorNode node{};
        int node_id{-1};
        int node_index{-1};
        int hint_index{-1};
        double time_s{0.0};
        orbitsim::Vec3 dv_rtn_mps{0.0, 0.0, 0.0};
        bool primary_body_auto{true};
        orbitsim::BodyId primary_body_id{orbitsim::kInvalidBodyId};
        bool select_added{true};
        bool defer_prediction_dirty{false};

        static KeplerManeuverCommand add_node(KeplerManeuverEditorNode node, bool select_added = true);
        static KeplerManeuverCommand clear_plan();
        static KeplerManeuverCommand select_node(int node_id);
        static KeplerManeuverCommand select_node_by_index(int node_index);
        static KeplerManeuverCommand ensure_selection();
        static KeplerManeuverCommand remove_node(int node_id, int hint_index = -1);
        static KeplerManeuverCommand prune_past_nodes(double current_time_s);
        static KeplerManeuverCommand set_node_time(int node_id,
                                                   double time_s,
                                                   bool defer_prediction_dirty = false);
        static KeplerManeuverCommand set_node_dv(int node_id,
                                                 const orbitsim::Vec3 &dv_rtn_mps,
                                                 bool defer_prediction_dirty = false);
        static KeplerManeuverCommand set_node_primary_body(int node_id,
                                                           bool primary_body_auto,
                                                           orbitsim::BodyId primary_body_id);
        static KeplerManeuverCommand sort_by_time();
        static KeplerManeuverCommand mark_plan_dirty();
    };

    struct KeplerManeuverCommandResult
    {
        bool applied{false};
        bool plan_changed{false};
        bool selection_changed{false};
        bool nodes_removed{false};
        bool plan_empty{false};
        bool prediction_dirty{false};
        int previous_selected_node_id{-1};
        int selected_node_id{-1};
        int added_node_id{-1};
        std::vector<int> removed_node_ids{};
    };
} // namespace Game
