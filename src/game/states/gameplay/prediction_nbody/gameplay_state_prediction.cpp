#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/maneuver_nbody/maneuver_prediction_bridge.h"
#include "game/states/gameplay/prediction_nbody/gameplay_prediction_adapter.h"
#include "game/states/gameplay/prediction_nbody/prediction_host_context_builder.h"
#include "game/states/gameplay/prediction_nbody/runtime/prediction_window_context_builder.h"

#include <utility>

namespace Game
{
    GameplayPredictionContext GameplayState::build_prediction_context()
    {
        return GameplayPredictionContext{
            .orbit = _orbit,
            .world = _world,
            .physics = _physics.get(),
            .physics_context = _physics_context.get(),
            .scenario_config = _scenario_config,
            .orbital_physics = _orbital_physics,
            .time_warp = _time_warp,
            .maneuver = _maneuver,
            .prediction = _prediction.get(),
            .fixed_time_s = _fixed_time_s,
            .current_sim_time_s = current_sim_time_s(),
            .debug_draw_enabled = _debug_draw_enabled,
            .resolve_maneuver_node_primary_body_id =
                    [this](const ManeuverNode &node, const double query_time_s) {
                        ManeuverPredictionBridge::Context context = build_maneuver_prediction_context();
                        return ManeuverPredictionBridge::resolve_node_primary_body_id(
                                context,
                                node,
                                query_time_s);
                    },
        };
    }

    GameplayPredictionAccess GameplayState::build_prediction_access()
    {
        return GameplayPredictionAccess{
            .build_context = [this]() {
                return build_prediction_context();
            },
            .prediction = *_prediction,
            .maneuver = _maneuver,
            .current_sim_time_s = [this]() {
                return current_sim_time_s();
            },
            .mark_prediction_dirty = [this]() {
                mark_prediction_dirty();
            },
        };
    }

    GameplayPredictionContext GameplayPredictionAdapter::context() const
    {
        return _access.build_context();
    }

    GameplayPredictionState &GameplayPredictionAdapter::prediction_draw_state()
    {
        return _access.prediction.state();
    }

    const GameplayPredictionState &GameplayPredictionAdapter::prediction_draw_state() const
    {
        return _access.prediction.state();
    }

    OrbitPlotBudgetSettings &GameplayPredictionAdapter::prediction_draw_budget()
    {
        return _access.prediction.budget();
    }

    const OrbitPlotBudgetSettings &GameplayPredictionAdapter::prediction_draw_budget() const
    {
        return _access.prediction.budget();
    }

    bool GameplayPredictionAdapter::prediction_subject_world_state(const PredictionSubjectKey key,
                                                                   WorldVec3 &out_pos_world,
                                                                   glm::dvec3 &out_vel_world,
                                                                   glm::vec3 &out_vel_local) const
    {
        return get_prediction_subject_world_state(key, out_pos_world, out_vel_world, out_vel_local);
    }

    PredictionSubjectKey GameplayPredictionAdapter::player_prediction_subject_key() const
    {
        return PredictionHostContextBuilder(context()).make_subject_state_provider().player_subject_key();
    }

    std::vector<PredictionSubjectDescriptor> GameplayPredictionAdapter::build_prediction_subject_descriptors() const
    {
        return PredictionHostContextBuilder(context()).make_subject_state_provider().build_subject_descriptors();
    }

    bool GameplayPredictionAdapter::get_player_world_state(WorldVec3 &out_pos_world,
                                                           glm::dvec3 &out_vel_world,
                                                           glm::vec3 &out_vel_local) const
    {
        return PredictionHostContextBuilder(context()).make_subject_state_provider().get_player_world_state(
                out_pos_world,
                out_vel_world,
                out_vel_local);
    }

    bool GameplayPredictionAdapter::get_orbiter_world_state(const OrbiterInfo &orbiter,
                                                            WorldVec3 &out_pos_world,
                                                            glm::dvec3 &out_vel_world,
                                                            glm::vec3 &out_vel_local) const
    {
        return PredictionHostContextBuilder(context()).make_subject_state_provider().get_orbiter_world_state(
                orbiter,
                out_pos_world,
                out_vel_world,
                out_vel_local);
    }

    bool GameplayPredictionAdapter::get_prediction_subject_world_state(PredictionSubjectKey key,
                                                                       WorldVec3 &out_pos_world,
                                                                       glm::dvec3 &out_vel_world,
                                                                       glm::vec3 &out_vel_local) const
    {
        return PredictionHostContextBuilder(context()).make_subject_state_provider().get_subject_world_state(
                key,
                out_pos_world,
                out_vel_world,
                out_vel_local);
    }

    void GameplayPredictionAdapter::rebuild_prediction_subjects()
    {
        _access.prediction.sync_subjects(PredictionHostContextBuilder(context()).build());
    }

    std::vector<PredictionSubjectKey> GameplayPredictionAdapter::collect_visible_prediction_subjects() const
    {
        return _access.prediction.collect_visible_subjects();
    }

#if defined(VULKAN_ENGINE_GAMEPLAY_TEST_ACCESS)
    double GameplayPredictionAdapter::prediction_future_window_s(const PredictionSubjectKey key) const
    {
        return PredictionWindowContextBuilder(context()).future_window_s(key);
    }

    double GameplayPredictionAdapter::maneuver_plan_horizon_s() const
    {
        return PredictionWindowContextBuilder(context()).maneuver_plan_horizon_s();
    }

    uint64_t GameplayPredictionAdapter::current_maneuver_plan_signature() const
    {
        return PredictionWindowContextBuilder(context()).current_maneuver_plan_signature();
    }
#endif

    PredictionTrackState *GameplayPredictionAdapter::find_prediction_track(PredictionSubjectKey key)
    {
        return _access.prediction.find_track(key);
    }

    const PredictionTrackState *GameplayPredictionAdapter::find_prediction_track(PredictionSubjectKey key) const
    {
        return _access.prediction.find_track(key);
    }

    PredictionTrackState *GameplayPredictionAdapter::active_prediction_track()
    {
        return _access.prediction.active_track();
    }

    const PredictionTrackState *GameplayPredictionAdapter::active_prediction_track() const
    {
        return _access.prediction.active_track();
    }

    PredictionTrackState *GameplayPredictionAdapter::player_prediction_track()
    {
        return _access.prediction.player_track(player_prediction_subject_key());
    }

    const PredictionTrackState *GameplayPredictionAdapter::player_prediction_track() const
    {
        return _access.prediction.player_track(player_prediction_subject_key());
    }

    OrbitPredictionCache *GameplayPredictionAdapter::effective_prediction_cache(PredictionTrackState *track)
    {
        return _access.prediction.effective_cache(track);
    }

    const OrbitPredictionCache *GameplayPredictionAdapter::effective_prediction_cache(const PredictionTrackState *track) const
    {
        return _access.prediction.effective_cache(track);
    }

    OrbitPredictionCache *GameplayPredictionAdapter::player_prediction_cache()
    {
        return _access.prediction.player_cache(player_prediction_subject_key());
    }

    const OrbitPredictionCache *GameplayPredictionAdapter::player_prediction_cache() const
    {
        return _access.prediction.player_cache(player_prediction_subject_key());
    }

    bool GameplayPredictionAdapter::prediction_subject_is_player(PredictionSubjectKey key) const
    {
        return PredictionHostContextBuilder(context()).make_subject_state_provider().subject_is_player(key);
    }

    bool GameplayPredictionAdapter::prediction_subject_supports_maneuvers(PredictionSubjectKey key) const
    {
        return PredictionHostContextBuilder(context()).make_subject_state_provider().subject_supports_maneuvers(key);
    }

    std::string GameplayPredictionAdapter::prediction_subject_label(PredictionSubjectKey key) const
    {
        // Build a UI label that reflects subject type and player ownership.
        const PredictionTrackState *track = find_prediction_track(key);
        if (!track)
        {
            return {};
        }

        if (key.kind == PredictionSubjectKind::Orbiter && prediction_subject_is_player(key))
        {
            return track->label + " (player)";
        }
        if (key.kind == PredictionSubjectKind::Celestial)
        {
            return track->label + " (celestial)";
        }
        return track->label;
    }

    glm::vec3 GameplayPredictionAdapter::prediction_subject_orbit_rgb(const PredictionSubjectKey key) const
    {
        return PredictionHostContextBuilder(context()).make_subject_state_provider().subject_orbit_rgb(key);
    }

    bool GameplayPredictionAdapter::prediction_subject_thrust_applied_this_tick(PredictionSubjectKey key) const
    {
        return PredictionHostContextBuilder(context()).make_subject_state_provider().subject_thrust_applied_this_tick(key);
    }

} // namespace Game
