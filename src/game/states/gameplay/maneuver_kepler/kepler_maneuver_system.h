#pragma once

#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_commands.h"
#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_node_resolver.h"
#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_plan.h"

#include <cstdint>
#include <span>

namespace Game
{
    // Owns authored Kepler maneuver nodes, display state, and prediction-facing node cache.
    class KeplerManeuverSystem
    {
    public:
        // Returns the editable maneuver plan.
        KeplerManeuverPlanState &plan();
        const KeplerManeuverPlanState &plan() const;
        // Returns the current gizmo interaction state.
        KeplerManeuverInteraction &interaction();
        const KeplerManeuverInteraction &interaction() const;
        // Returns normalized nodes consumed by Kepler prediction.
        std::span<const KeplerManeuverNode> prediction_nodes() const;
        // Returns resolved node positions and bases used by the gizmo.
        std::span<const KeplerManeuverNodeDisplayState> node_display_states() const;
        // Finds the resolved display state for one node id.
        const KeplerManeuverNodeDisplayState *find_node_display_state(int node_id) const;
        // Returns the latest display-state resolve summary.
        const KeplerManeuverNodeResolveResult &last_node_resolve_result() const;
        // Increments whenever prediction-affecting plan data changes.
        uint64_t revision() const;

        // Clears plan, interaction, cached prediction nodes, and display states.
        void reset_session();
        // Clears only transient gizmo interaction.
        void clear_interaction();
        // Clears only resolved display data.
        void clear_node_display_states();
        // Sets the external revision value, typically during state restore.
        void set_revision(uint64_t revision);
        // Advances the maneuver revision and returns the new value.
        uint64_t increment_revision();
        // Rebuilds display states from the latest Kepler prediction result.
        KeplerManeuverNodeResolveResult resolve_node_display_states(
                const KeplerPredictionState &prediction);
        // Applies a single editor/gizmo command and reports downstream dirty flags.
        KeplerManeuverCommandResult apply_command(const KeplerManeuverCommand &command);

    private:
        // Rebuilds the prediction-facing node cache from the editable plan.
        void sync_prediction_nodes();
        // Mirrors plan selection into already-resolved display states.
        void sync_display_selection();
        // Clears drag/hover state when its target node was removed.
        void clear_removed_interaction(const std::vector<int> &removed_node_ids);

        KeplerManeuverPlanState _plan{};
        KeplerManeuverInteraction _interaction{};
        std::vector<KeplerManeuverNode> _prediction_nodes{};
        std::vector<KeplerManeuverNodeDisplayState> _node_display_states{};
        KeplerManeuverNodeResolveResult _last_node_resolve_result{};
        uint64_t _revision{0};
    };
} // namespace Game
