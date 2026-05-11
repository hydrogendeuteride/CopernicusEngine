#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_commands.h"

namespace Game
{
    KeplerManeuverCommand KeplerManeuverCommand::add_node(KeplerManeuverEditorNode node,
                                                          const bool select_added)
    {
        KeplerManeuverCommand command{};
        command.kind = KeplerManeuverCommandKind::AddNode;
        command.node = node;
        command.select_added = select_added;
        return command;
    }

    KeplerManeuverCommand KeplerManeuverCommand::clear_plan()
    {
        KeplerManeuverCommand command{};
        command.kind = KeplerManeuverCommandKind::ClearPlan;
        return command;
    }

    KeplerManeuverCommand KeplerManeuverCommand::select_node(const int node_id)
    {
        KeplerManeuverCommand command{};
        command.kind = KeplerManeuverCommandKind::SelectNode;
        command.node_id = node_id;
        return command;
    }

    KeplerManeuverCommand KeplerManeuverCommand::select_node_by_index(const int node_index)
    {
        KeplerManeuverCommand command{};
        command.kind = KeplerManeuverCommandKind::SelectNodeByIndex;
        command.node_index = node_index;
        return command;
    }

    KeplerManeuverCommand KeplerManeuverCommand::ensure_selection()
    {
        KeplerManeuverCommand command{};
        command.kind = KeplerManeuverCommandKind::EnsureSelection;
        return command;
    }

    KeplerManeuverCommand KeplerManeuverCommand::remove_node(const int node_id,
                                                             const int hint_index)
    {
        KeplerManeuverCommand command{};
        command.kind = KeplerManeuverCommandKind::RemoveNode;
        command.node_id = node_id;
        command.hint_index = hint_index;
        return command;
    }

    KeplerManeuverCommand KeplerManeuverCommand::set_node_time(const int node_id,
                                                               const double time_s,
                                                               const bool defer_prediction_dirty)
    {
        KeplerManeuverCommand command{};
        command.kind = KeplerManeuverCommandKind::SetNodeTime;
        command.node_id = node_id;
        command.time_s = time_s;
        command.defer_prediction_dirty = defer_prediction_dirty;
        return command;
    }

    KeplerManeuverCommand KeplerManeuverCommand::set_node_dv(const int node_id,
                                                             const orbitsim::Vec3 &dv_rtn_mps,
                                                             const bool defer_prediction_dirty)
    {
        KeplerManeuverCommand command{};
        command.kind = KeplerManeuverCommandKind::SetNodeDv;
        command.node_id = node_id;
        command.dv_rtn_mps = dv_rtn_mps;
        command.defer_prediction_dirty = defer_prediction_dirty;
        return command;
    }

    KeplerManeuverCommand KeplerManeuverCommand::set_node_primary_body(
            const int node_id,
            const bool primary_body_auto,
            const orbitsim::BodyId primary_body_id)
    {
        KeplerManeuverCommand command{};
        command.kind = KeplerManeuverCommandKind::SetNodePrimaryBody;
        command.node_id = node_id;
        command.primary_body_auto = primary_body_auto;
        command.primary_body_id = primary_body_id;
        return command;
    }

    KeplerManeuverCommand KeplerManeuverCommand::sort_by_time()
    {
        KeplerManeuverCommand command{};
        command.kind = KeplerManeuverCommandKind::SortByTime;
        return command;
    }

    KeplerManeuverCommand KeplerManeuverCommand::mark_plan_dirty()
    {
        KeplerManeuverCommand command{};
        command.kind = KeplerManeuverCommandKind::MarkPlanDirty;
        return command;
    }
} // namespace Game
