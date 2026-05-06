#pragma once

#include "core/world.h"
#include "game/states/gameplay/maneuver_nbody/maneuver_system.h"
#include "game/states/gameplay/prediction_nbody/gameplay_prediction_access.h"
#include "orbitsim/types.hpp"

#include <functional>
#include <vector>

namespace orbitsim
{
    struct TrajectorySample;
} // namespace orbitsim

namespace Game
{
    class GameWorld;
    class OrbitalPhysicsSystem;
    class OrbitalRuntimeSystem;
    class PredictionSystem;
    struct GameStateContext;
    struct OrbitPredictionCache;

    struct ManeuverPredictionBridgeContext
    {
        ManeuverSystem &maneuver;
        PredictionSystem &prediction;
        GameplayPredictionAccess prediction_access;
        const OrbitalRuntimeSystem &orbit;
        const OrbitalPhysicsSystem &orbital_physics;
        const GameWorld &world;
        std::function<ManeuverCommandResult(const ManeuverCommand &)> apply_maneuver_command;
        std::function<double()> current_sim_time_s;
    };

    class ManeuverPredictionBridge
    {
    public:
        using Context = ManeuverPredictionBridgeContext;

        static void begin_node_dv_edit_preview(Context &context, int node_id);
        static void update_node_dv_edit_preview(Context &context, int node_id);
        static void finish_node_dv_edit_preview(Context &context, bool changed);
        static void begin_node_time_edit_preview(Context &context, int node_id, double previous_time_s);
        static void update_node_time_edit_preview(Context &context, int node_id, double previous_time_s);
        static void finish_node_time_edit_preview(Context &context, bool changed);

        static orbitsim::BodyId resolve_node_primary_body_id(const Context &context,
                                                             const ManeuverNode &node,
                                                             double query_time_s);
        static WorldVec3 compute_align_delta(const Context &context,
                                             GameStateContext &ctx,
                                             const OrbitPredictionCache &cache,
                                             const std::vector<orbitsim::TrajectorySample> &traj_base);
    };
} // namespace Game
