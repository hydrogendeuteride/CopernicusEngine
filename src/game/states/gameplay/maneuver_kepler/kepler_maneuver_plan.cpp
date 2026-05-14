#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_plan.h"

#include <algorithm>
#include <cmath>

namespace Game
{
    namespace
    {
        bool same_vec3(const orbitsim::Vec3 &a, const orbitsim::Vec3 &b)
        {
            return a.x == b.x && a.y == b.y && a.z == b.z;
        }

        bool valid_editor_node(const KeplerManeuverEditorNode &node)
        {
            return std::isfinite(node.time_s) &&
                   kepler_finite_vec3(node.dv_rtn_mps) &&
                   (node.primary_body_auto || node.primary_body_id != orbitsim::kInvalidBodyId);
        }
    } // namespace

    KeplerManeuverNode KeplerManeuverEditorNode::to_prediction_node() const
    {
        return KeplerManeuverNode{
                .node_id = id,
                .t_s = time_s,
                .primary_body_id = primary_body_auto ? orbitsim::kInvalidBodyId : primary_body_id,
                .dv_rtn_mps = dv_rtn_mps,
        };
    }

    KeplerManeuverEditorNode *KeplerManeuverPlanState::find_node(const int id)
    {
        for (KeplerManeuverEditorNode &node : nodes)
        {
            if (node.id == id)
            {
                return &node;
            }
        }
        return nullptr;
    }

    const KeplerManeuverEditorNode *KeplerManeuverPlanState::find_node(const int id) const
    {
        for (const KeplerManeuverEditorNode &node : nodes)
        {
            if (node.id == id)
            {
                return &node;
            }
        }
        return nullptr;
    }

    bool KeplerManeuverPlanState::sort_by_time()
    {
        std::vector<int> before_ids;
        before_ids.reserve(nodes.size());
        for (const KeplerManeuverEditorNode &node : nodes)
        {
            before_ids.push_back(node.id);
        }

        std::stable_sort(nodes.begin(), nodes.end(), [](const KeplerManeuverEditorNode &a,
                                                        const KeplerManeuverEditorNode &b) {
            return a.time_s < b.time_s;
        });

        for (std::size_t i = 0; i < nodes.size(); ++i)
        {
            if (nodes[i].id != before_ids[i])
            {
                return true;
            }
        }
        return false;
    }

    std::vector<KeplerManeuverNode> KeplerManeuverPlanState::to_prediction_nodes() const
    {
        std::vector<KeplerManeuverNode> out;
        out.reserve(nodes.size());
        for (const KeplerManeuverEditorNode &node : nodes)
        {
            out.push_back(node.to_prediction_node());
        }
        return out;
    }

    KeplerManeuverPlanModel::KeplerManeuverPlanModel(KeplerManeuverPlanState &state)
        : _state(state)
    {
    }

    KeplerManeuverPlanState &KeplerManeuverPlanModel::state()
    {
        return _state;
    }

    const KeplerManeuverPlanState &KeplerManeuverPlanModel::state() const
    {
        return _state;
    }

    std::vector<KeplerManeuverEditorNode> &KeplerManeuverPlanModel::nodes()
    {
        return _state.nodes;
    }

    const std::vector<KeplerManeuverEditorNode> &KeplerManeuverPlanModel::nodes() const
    {
        return _state.nodes;
    }

    int KeplerManeuverPlanModel::selected_node_id() const
    {
        return _state.selected_node_id;
    }

    int KeplerManeuverPlanModel::next_node_id() const
    {
        return _state.next_node_id;
    }

    bool KeplerManeuverPlanModel::empty() const
    {
        return _state.nodes.empty();
    }

    std::size_t KeplerManeuverPlanModel::size() const
    {
        return _state.nodes.size();
    }

    KeplerManeuverEditorNode *KeplerManeuverPlanModel::find_node(const int node_id)
    {
        return _state.find_node(node_id);
    }

    const KeplerManeuverEditorNode *KeplerManeuverPlanModel::find_node(const int node_id) const
    {
        return _state.find_node(node_id);
    }

    int KeplerManeuverPlanModel::find_node_index(const int node_id) const
    {
        for (std::size_t i = 0; i < _state.nodes.size(); ++i)
        {
            if (_state.nodes[i].id == node_id)
            {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    int KeplerManeuverPlanModel::add_node(KeplerManeuverEditorNode node, const bool select_added)
    {
        if (!valid_editor_node(node))
        {
            return -1;
        }

        if (node.id < 0)
        {
            node.id = _state.next_node_id++;
        }
        else
        {
            _state.next_node_id = std::max(_state.next_node_id, node.id + 1);
        }

        const int node_id = node.id;
        _state.nodes.push_back(node);
        if (select_added)
        {
            _state.selected_node_id = node_id;
        }
        sort_by_time();
        return node_id;
    }

    bool KeplerManeuverPlanModel::clear()
    {
        if (_state.nodes.empty() && _state.selected_node_id < 0)
        {
            return false;
        }

        _state.nodes.clear();
        _state.selected_node_id = -1;
        return true;
    }

    bool KeplerManeuverPlanModel::select_node(const int node_id)
    {
        if (_state.selected_node_id == node_id)
        {
            return false;
        }
        if (node_id >= 0 && !_state.find_node(node_id))
        {
            return false;
        }

        _state.selected_node_id = node_id;
        return true;
    }

    bool KeplerManeuverPlanModel::select_index(const int node_index)
    {
        if (node_index < 0 || node_index >= static_cast<int>(_state.nodes.size()))
        {
            return false;
        }
        return select_node(_state.nodes[static_cast<std::size_t>(node_index)].id);
    }

    bool KeplerManeuverPlanModel::ensure_valid_selection()
    {
        if (_state.selected_node_id >= 0 && _state.find_node(_state.selected_node_id))
        {
            return false;
        }

        const int next_selection = _state.nodes.empty() ? -1 : _state.nodes.front().id;
        if (_state.selected_node_id == next_selection)
        {
            return false;
        }

        _state.selected_node_id = next_selection;
        return true;
    }

    bool KeplerManeuverPlanModel::set_node_time(const int node_id, const double time_s)
    {
        KeplerManeuverEditorNode *node = _state.find_node(node_id);
        if (!node || !std::isfinite(time_s) || node->time_s == time_s)
        {
            return false;
        }

        node->time_s = time_s;
        sort_by_time();
        return true;
    }

    bool KeplerManeuverPlanModel::set_node_dv(const int node_id, const orbitsim::Vec3 &dv_rtn_mps)
    {
        KeplerManeuverEditorNode *node = _state.find_node(node_id);
        if (!node || !kepler_finite_vec3(dv_rtn_mps) || same_vec3(node->dv_rtn_mps, dv_rtn_mps))
        {
            return false;
        }

        node->dv_rtn_mps = dv_rtn_mps;
        return true;
    }

    bool KeplerManeuverPlanModel::set_node_primary_body(const int node_id,
                                                        const bool primary_body_auto,
                                                        const orbitsim::BodyId primary_body_id)
    {
        KeplerManeuverEditorNode *node = _state.find_node(node_id);
        if (!node)
        {
            return false;
        }

        if (node->primary_body_auto == primary_body_auto && node->primary_body_id == primary_body_id)
        {
            return false;
        }

        node->primary_body_auto = primary_body_auto;
        node->primary_body_id = primary_body_id;
        return true;
    }

    bool KeplerManeuverPlanModel::sort_by_time()
    {
        return _state.sort_by_time();
    }

    KeplerManeuverNodeRemovalResult KeplerManeuverPlanModel::remove_node(const int node_id,
                                                                         const int hint_index)
    {
        KeplerManeuverNodeRemovalResult result{};
        const auto before_size = _state.nodes.size();
        const bool removed_selected = _state.selected_node_id == node_id;

        _state.nodes.erase(
                std::remove_if(_state.nodes.begin(),
                               _state.nodes.end(),
                               [&](const KeplerManeuverEditorNode &node) { return node.id == node_id; }),
                _state.nodes.end());
        result.removed = _state.nodes.size() != before_size;
        if (!result.removed)
        {
            return result;
        }

        result.removed_selected = removed_selected;
        result.removed_node_ids.push_back(node_id);
        if (removed_selected)
        {
            if (_state.nodes.empty())
            {
                _state.selected_node_id = -1;
            }
            else if (hint_index >= 0)
            {
                const int new_index = std::clamp(hint_index, 0, static_cast<int>(_state.nodes.size()) - 1);
                _state.selected_node_id = _state.nodes[static_cast<std::size_t>(new_index)].id;
            }
            else
            {
                _state.selected_node_id = _state.nodes.front().id;
            }
        }
        result.plan_empty = _state.nodes.empty();
        return result;
    }

    KeplerManeuverNodeRemovalResult KeplerManeuverPlanModel::prune_past_nodes(
            const double current_time_s)
    {
        KeplerManeuverNodeRemovalResult result{};
        if (!std::isfinite(current_time_s) || _state.nodes.empty())
        {
            return result;
        }

        const int previous_selected_node_id = _state.selected_node_id;
        for (const KeplerManeuverEditorNode &node : _state.nodes)
        {
            if (std::isfinite(node.time_s) && node.time_s <= current_time_s)
            {
                result.removed_node_ids.push_back(node.id);
                if (node.id == previous_selected_node_id)
                {
                    result.removed_selected = true;
                }
            }
        }
        if (result.removed_node_ids.empty())
        {
            return result;
        }

        _state.nodes.erase(
                std::remove_if(_state.nodes.begin(),
                               _state.nodes.end(),
                               [current_time_s](const KeplerManeuverEditorNode &node) {
                                   return std::isfinite(node.time_s) &&
                                          node.time_s <= current_time_s;
                               }),
                _state.nodes.end());

        result.removed = true;
        result.plan_empty = _state.nodes.empty();
        if (result.plan_empty)
        {
            _state.selected_node_id = -1;
        }
        else if (result.removed_selected || !_state.find_node(_state.selected_node_id))
        {
            _state.selected_node_id = _state.nodes.front().id;
            result.removed_selected = true;
        }
        return result;
    }
} // namespace Game
