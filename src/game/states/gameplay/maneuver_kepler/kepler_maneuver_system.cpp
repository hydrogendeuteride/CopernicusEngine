#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_system.h"

#include <algorithm>
#include <utility>

namespace Game
{
    namespace
    {
        bool contains_node_id(const std::vector<int> &ids, const int node_id)
        {
            return std::find(ids.begin(), ids.end(), node_id) != ids.end();
        }

        void mark_plan_changed(KeplerManeuverCommandResult &result,
                               const KeplerManeuverCommand &command)
        {
            result.plan_changed = true;
            result.prediction_dirty = !command.defer_prediction_dirty;
        }
    } // namespace

    KeplerManeuverPlanState &KeplerManeuverSystem::plan()
    {
        return _plan;
    }

    const KeplerManeuverPlanState &KeplerManeuverSystem::plan() const
    {
        return _plan;
    }

    KeplerManeuverInteraction &KeplerManeuverSystem::interaction()
    {
        return _interaction;
    }

    const KeplerManeuverInteraction &KeplerManeuverSystem::interaction() const
    {
        return _interaction;
    }

    std::span<const KeplerManeuverNode> KeplerManeuverSystem::prediction_nodes() const
    {
        return std::span<const KeplerManeuverNode>(_prediction_nodes.data(), _prediction_nodes.size());
    }

    std::span<const KeplerManeuverNodeDisplayState> KeplerManeuverSystem::node_display_states() const
    {
        return std::span<const KeplerManeuverNodeDisplayState>(_node_display_states.data(),
                                                               _node_display_states.size());
    }

    const KeplerManeuverNodeDisplayState *KeplerManeuverSystem::find_node_display_state(
            const int node_id) const
    {
        const auto it = std::find_if(_node_display_states.begin(),
                                     _node_display_states.end(),
                                     [node_id](const KeplerManeuverNodeDisplayState &state) {
                                         return state.node_id == node_id;
                                     });
        return it != _node_display_states.end() ? &(*it) : nullptr;
    }

    const KeplerManeuverNodeResolveResult &KeplerManeuverSystem::last_node_resolve_result() const
    {
        return _last_node_resolve_result;
    }

    uint64_t KeplerManeuverSystem::revision() const
    {
        return _revision;
    }

    void KeplerManeuverSystem::reset_session()
    {
        _plan.nodes.clear();
        _plan.selected_node_id = -1;
        _plan.next_node_id = 0;
        _interaction = {};
        _prediction_nodes.clear();
        clear_node_display_states();
    }

    void KeplerManeuverSystem::clear_interaction()
    {
        _interaction = {};
    }

    void KeplerManeuverSystem::clear_node_display_states()
    {
        _node_display_states.clear();
        _last_node_resolve_result = {};
    }

    void KeplerManeuverSystem::set_revision(const uint64_t revision)
    {
        _revision = revision;
    }

    uint64_t KeplerManeuverSystem::increment_revision()
    {
        return ++_revision;
    }

    void KeplerManeuverSystem::sync_prediction_nodes()
    {
        _prediction_nodes = _plan.to_prediction_nodes();
    }

    void KeplerManeuverSystem::sync_display_selection()
    {
        for (KeplerManeuverNodeDisplayState &state : _node_display_states)
        {
            state.selected = state.node_id == _plan.selected_node_id;
        }
    }

    void KeplerManeuverSystem::clear_removed_interaction(const std::vector<int> &removed_node_ids)
    {
        if (contains_node_id(removed_node_ids, _interaction.node_id))
        {
            _interaction = {};
        }
    }

    KeplerManeuverNodeResolveResult KeplerManeuverSystem::resolve_node_display_states(
            const KeplerPredictionState &prediction)
    {
        _last_node_resolve_result =
                resolve_kepler_maneuver_node_display_states(_plan,
                                                            prediction,
                                                            _node_display_states);
        return _last_node_resolve_result;
    }

    KeplerManeuverCommandResult KeplerManeuverSystem::apply_command(const KeplerManeuverCommand &command)
    {
        KeplerManeuverPlanModel model(_plan);
        KeplerManeuverCommandResult result{};
        result.previous_selected_node_id = model.selected_node_id();
        result.selected_node_id = model.selected_node_id();

        switch (command.kind)
        {
            case KeplerManeuverCommandKind::None:
                break;

            case KeplerManeuverCommandKind::AddNode:
                result.added_node_id = model.add_node(command.node, command.select_added);
                result.applied = true;
                mark_plan_changed(result, command);
                break;

            case KeplerManeuverCommandKind::ClearPlan:
                for (const KeplerManeuverEditorNode &node : model.nodes())
                {
                    result.removed_node_ids.push_back(node.id);
                }
                result.applied = model.clear();
                if (result.applied)
                {
                    result.nodes_removed = !result.removed_node_ids.empty();
                    result.plan_empty = true;
                    mark_plan_changed(result, command);
                }
                break;

            case KeplerManeuverCommandKind::SelectNode:
                result.applied = model.select_node(command.node_id);
                result.selection_changed = result.applied;
                break;

            case KeplerManeuverCommandKind::SelectNodeByIndex:
                result.applied = model.select_index(command.node_index);
                result.selection_changed = result.applied;
                break;

            case KeplerManeuverCommandKind::EnsureSelection:
                result.applied = model.ensure_valid_selection();
                result.selection_changed = result.applied;
                break;

            case KeplerManeuverCommandKind::RemoveNode:
            {
                KeplerManeuverNodeRemovalResult removal =
                        model.remove_node(command.node_id, command.hint_index);
                result.applied = removal.removed;
                if (result.applied)
                {
                    result.nodes_removed = true;
                    result.plan_empty = removal.plan_empty;
                    result.removed_node_ids = std::move(removal.removed_node_ids);
                    result.selection_changed = removal.removed_selected;
                    mark_plan_changed(result, command);
                }
                break;
            }

            case KeplerManeuverCommandKind::SetNodeTime:
                result.applied = model.set_node_time(command.node_id, command.time_s);
                if (result.applied)
                {
                    mark_plan_changed(result, command);
                }
                break;

            case KeplerManeuverCommandKind::SetNodeDv:
                result.applied = model.set_node_dv(command.node_id, command.dv_rtn_mps);
                if (result.applied)
                {
                    mark_plan_changed(result, command);
                }
                break;

            case KeplerManeuverCommandKind::SetNodePrimaryBody:
                result.applied = model.set_node_primary_body(command.node_id,
                                                             command.primary_body_auto,
                                                             command.primary_body_id);
                if (result.applied)
                {
                    mark_plan_changed(result, command);
                }
                break;

            case KeplerManeuverCommandKind::SortByTime:
                result.applied = model.sort_by_time();
                if (result.applied)
                {
                    mark_plan_changed(result, command);
                }
                break;

            case KeplerManeuverCommandKind::MarkPlanDirty:
                result.applied = true;
                mark_plan_changed(result, command);
                break;
        }

        result.selected_node_id = model.selected_node_id();
        if (result.selected_node_id != result.previous_selected_node_id)
        {
            result.selection_changed = true;
        }

        if (result.nodes_removed)
        {
            clear_removed_interaction(result.removed_node_ids);
        }
        if (result.plan_changed)
        {
            sync_prediction_nodes();
            clear_node_display_states();
            increment_revision();
        }
        else if (result.selection_changed)
        {
            sync_display_selection();
        }
        return result;
    }
} // namespace Game
