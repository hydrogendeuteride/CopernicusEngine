#pragma once

#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_types.h"

#include <cstddef>
#include <vector>

namespace Game
{
    struct KeplerManeuverNodeRemovalResult
    {
        bool removed{false};
        bool removed_selected{false};
        bool plan_empty{false};
        std::vector<int> removed_node_ids{};
    };

    class KeplerManeuverPlanModel
    {
    public:
        explicit KeplerManeuverPlanModel(KeplerManeuverPlanState &state);

        [[nodiscard]] KeplerManeuverPlanState &state();
        [[nodiscard]] const KeplerManeuverPlanState &state() const;
        [[nodiscard]] std::vector<KeplerManeuverEditorNode> &nodes();
        [[nodiscard]] const std::vector<KeplerManeuverEditorNode> &nodes() const;
        [[nodiscard]] int selected_node_id() const;
        [[nodiscard]] int next_node_id() const;
        [[nodiscard]] bool empty() const;
        [[nodiscard]] std::size_t size() const;

        [[nodiscard]] KeplerManeuverEditorNode *find_node(int node_id);
        [[nodiscard]] const KeplerManeuverEditorNode *find_node(int node_id) const;
        [[nodiscard]] int find_node_index(int node_id) const;

        int add_node(KeplerManeuverEditorNode node, bool select_added);
        bool clear();
        bool select_node(int node_id);
        bool select_index(int node_index);
        bool ensure_valid_selection();
        bool set_node_time(int node_id, double time_s);
        bool set_node_dv(int node_id, const orbitsim::Vec3 &dv_rtn_mps);
        bool set_node_primary_body(int node_id, bool primary_body_auto, orbitsim::BodyId primary_body_id);
        bool sort_by_time();
        KeplerManeuverNodeRemovalResult remove_node(int node_id, int hint_index);
        KeplerManeuverNodeRemovalResult prune_past_nodes(double current_time_s);

    private:
        KeplerManeuverPlanState &_state;
    };
} // namespace Game
