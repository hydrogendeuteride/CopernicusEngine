#pragma once

#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_commands.h"
#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_node_resolver.h"
#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_plan.h"

#include <cstdint>
#include <span>

namespace Game
{
    class KeplerManeuverSystem
    {
    public:
        [[nodiscard]] KeplerManeuverPlanState &plan();
        [[nodiscard]] const KeplerManeuverPlanState &plan() const;
        [[nodiscard]] KeplerManeuverInteraction &interaction();
        [[nodiscard]] const KeplerManeuverInteraction &interaction() const;
        [[nodiscard]] std::span<const KeplerManeuverNode> prediction_nodes() const;
        [[nodiscard]] std::span<const KeplerManeuverNodeDisplayState> node_display_states() const;
        [[nodiscard]] const KeplerManeuverNodeDisplayState *find_node_display_state(int node_id) const;
        [[nodiscard]] const KeplerManeuverNodeResolveResult &last_node_resolve_result() const;
        [[nodiscard]] uint64_t revision() const;

        void reset_session();
        void clear_interaction();
        void clear_node_display_states();
        void set_revision(uint64_t revision);
        uint64_t increment_revision();
        KeplerManeuverNodeResolveResult resolve_node_display_states(
                const KeplerPredictionState &prediction);
        KeplerManeuverCommandResult apply_command(const KeplerManeuverCommand &command);

    private:
        void sync_prediction_nodes();
        void sync_display_selection();
        void clear_removed_interaction(const std::vector<int> &removed_node_ids);

        KeplerManeuverPlanState _plan{};
        KeplerManeuverInteraction _interaction{};
        std::vector<KeplerManeuverNode> _prediction_nodes{};
        std::vector<KeplerManeuverNodeDisplayState> _node_display_states{};
        KeplerManeuverNodeResolveResult _last_node_resolve_result{};
        uint64_t _revision{0};
    };
} // namespace Game
