#include "game/states/gameplay/maneuver_nbody/maneuver_prediction_bridge.h"

#include "game/game_world.h"
#include "game/state/game_state.h"
#include "game/states/gameplay/maneuver_nbody/gameplay_state_maneuver_util.h"
#include "game/states/gameplay/maneuver_nbody/maneuver_commands.h"
#include "game/states/gameplay/orbital_physics_system.h"
#include "game/states/gameplay/orbital_runtime_system.h"
#include "game/states/gameplay/prediction_nbody/gameplay_prediction_adapter.h"
#include "game/states/gameplay/prediction_nbody/prediction_frame_context_builder.h"
#include "game/states/gameplay/prediction_nbody/prediction_host_context_builder.h"
#include "game/states/gameplay/prediction_nbody/runtime/prediction_window_context_builder.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace Game
{
    void ManeuverPredictionBridge::begin_node_dv_edit_preview(Context &context, const int node_id)
    {
        if (!context.maneuver.begin_dv_edit_preview(node_id))
        {
            return;
        }

        if (PredictionTrackState *track = context.prediction.active_track())
        {
            PredictionWindowContextBuilder(context.prediction_access.build_context())
                    .refresh_preview_anchor(*track, context.current_sim_time_s(), true);
        }
    }

    void ManeuverPredictionBridge::update_node_dv_edit_preview(Context &context, const int node_id)
    {
        begin_node_dv_edit_preview(context, node_id);
        if (!context.maneuver.mark_edit_preview_changed(ManeuverNodeEditPreview::State::EditingDv, node_id))
        {
            return;
        }

        if (PredictionTrackState *track = context.prediction.active_track())
        {
            context.prediction.mark_maneuver_preview_dirty(*track);
            context.prediction.sync_visible_dirty_flag(context.prediction.collect_visible_subjects());
        }
    }

    void ManeuverPredictionBridge::finish_node_dv_edit_preview(Context &context, const bool changed)
    {
        const bool preview_changed =
                context.maneuver.finish_edit_preview(ManeuverNodeEditPreview::State::EditingDv, changed);
        if (!preview_changed)
        {
            return;
        }

        if (PredictionTrackState *track = context.prediction.active_track())
        {
            context.prediction.await_maneuver_preview_full_refine(*track, context.current_sim_time_s());
        }
        (void) context.apply_maneuver_command(ManeuverCommand::mark_plan_dirty());
    }

    void ManeuverPredictionBridge::begin_node_time_edit_preview(Context &context,
                                                                const int node_id,
                                                                const double previous_time_s)
    {
        if (!context.maneuver.begin_time_edit_preview(node_id, previous_time_s))
        {
            return;
        }

        if (PredictionTrackState *track = context.prediction.active_track())
        {
            PredictionWindowContextBuilder(context.prediction_access.build_context())
                    .refresh_preview_anchor(*track, context.current_sim_time_s(), true);
        }
    }

    void ManeuverPredictionBridge::update_node_time_edit_preview(Context &context,
                                                                 const int node_id,
                                                                 const double previous_time_s)
    {
        begin_node_time_edit_preview(context, node_id, previous_time_s);
        if (!context.maneuver.mark_edit_preview_changed(ManeuverNodeEditPreview::State::EditingTime, node_id))
        {
            return;
        }

        if (PredictionTrackState *track = context.prediction.active_track())
        {
            context.prediction.mark_maneuver_preview_dirty(*track);
            context.prediction.sync_visible_dirty_flag(context.prediction.collect_visible_subjects());
        }
    }

    void ManeuverPredictionBridge::finish_node_time_edit_preview(Context &context, const bool changed)
    {
        const bool preview_changed =
                context.maneuver.finish_edit_preview(ManeuverNodeEditPreview::State::EditingTime, changed);
        if (!preview_changed)
        {
            return;
        }

        if (PredictionTrackState *track = context.prediction.active_track())
        {
            context.prediction.await_maneuver_preview_full_refine(*track, context.current_sim_time_s());
        }
        (void) context.apply_maneuver_command(ManeuverCommand::mark_plan_dirty());
    }

    orbitsim::BodyId ManeuverPredictionBridge::resolve_node_primary_body_id(const Context &context,
                                                                            const ManeuverNode &node,
                                                                            const double query_time_s)
    {
        if (!node.primary_body_auto && node.primary_body_id != orbitsim::kInvalidBodyId)
        {
            return node.primary_body_id;
        }

        const PredictionSubjectKey player_subject =
                PredictionHostContextBuilder(context.prediction_access.build_context())
                        .make_subject_state_provider()
                        .player_subject_key();
        const PredictionTrackState *player_track =
                context.prediction.player_track(player_subject);
        const OrbitPredictionCache *player_cache =
                context.prediction.effective_cache(player_track);
        PredictionFrameContextBuilder prediction_frame(context.prediction_access.build_context());
        if (player_cache)
        {
            const OrbitPredictionCache &cache = *player_cache;
            const auto &traj =
                    cache.solver.planned.trajectory_inertial.size() >= 2
                            ? cache.solver.planned.trajectory_inertial
                            : cache.solver.resolved_trajectory_inertial();
            const auto &bodies = cache.solver.resolved_massive_bodies();

            orbitsim::State sc_state{};
            if (!traj.empty() &&
                prediction_frame.sample_prediction_inertial_state(traj, query_time_s, sc_state) &&
                !bodies.empty())
            {
                const orbitsim::BodyId preferred_body_id =
                        node.primary_body_auto ? orbitsim::kInvalidBodyId : node.primary_body_id;
                const orbitsim::BodyId primary_body_id = prediction_frame.select_prediction_primary_body_id(
                        bodies,
                        &cache,
                        sc_state.position_m,
                        query_time_s,
                        preferred_body_id);
                if (primary_body_id != orbitsim::kInvalidBodyId)
                {
                    return primary_body_id;
                }
            }

            if (!cache.solver.resolved_trajectory_inertial().empty())
            {
                const orbitsim::BodyId analysis_body_id =
                        prediction_frame.resolve_prediction_analysis_body_id(cache, player_track->key, query_time_s, node.primary_body_id);
                if (analysis_body_id != orbitsim::kInvalidBodyId)
                {
                    return analysis_body_id;
                }
            }
        }

        if (node.primary_body_id != orbitsim::kInvalidBodyId)
        {
            return node.primary_body_id;
        }

        if (context.orbit.scenario_owner() && context.orbit.scenario_owner()->world_reference_body())
        {
            return context.orbit.scenario_owner()->world_reference_body()->sim_id;
        }

        return orbitsim::kInvalidBodyId;
    }

    WorldVec3 ManeuverPredictionBridge::compute_align_delta(const Context &context,
                                                            GameStateContext &ctx,
                                                            const OrbitPredictionCache &cache,
                                                            const std::vector<orbitsim::TrajectorySample> &traj_base)
    {
        if (traj_base.size() < 2)
        {
            return WorldVec3(0.0, 0.0, 0.0);
        }

        const float alpha_f = std::clamp(ctx.interpolation_alpha(), 0.0f, 1.0f);
        const double interp_dt_s =
                (context.orbital_physics.last_sim_step_dt_s() > 0.0)
                    ? context.orbital_physics.last_sim_step_dt_s()
                    : static_cast<double>(ctx.fixed_delta_time());
        double align_now_s = context.current_sim_time_s();
        if (std::isfinite(interp_dt_s) && interp_dt_s > 0.0)
        {
            align_now_s -= (1.0 - static_cast<double>(alpha_f)) * interp_dt_s;
        }
        align_now_s = std::clamp(align_now_s, traj_base.front().t_s, traj_base.back().t_s);

        auto it_align = std::lower_bound(traj_base.cbegin(), traj_base.cend(), align_now_s,
                                         [](const orbitsim::TrajectorySample &s, double t) { return s.t_s < t; });
        const std::size_t align_hi = static_cast<std::size_t>(std::distance(traj_base.cbegin(), it_align));
        if (align_hi >= traj_base.size())
        {
            return WorldVec3(0.0, 0.0, 0.0);
        }

        GameplayPredictionAdapter prediction(context.prediction_access);
        WorldVec3 predicted_now = (align_hi > 0)
                                      ? prediction.prediction_sample_hermite_world(cache, traj_base[align_hi - 1],
                                                                                   traj_base[align_hi], align_now_s, align_now_s)
                                      : prediction.prediction_sample_position_world(cache, traj_base.front(), align_now_s);

        WorldVec3 ship_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 ship_vel_world(0.0);
        glm::vec3 ship_vel_local_f(0.0f);
        if (!PredictionHostContextBuilder(context.prediction_access.build_context())
                     .make_subject_state_provider()
                     .get_player_world_state(
                    ship_pos_world,
                    ship_vel_world,
                    ship_vel_local_f))
        {
            return WorldVec3(0.0, 0.0, 0.0);
        }

        const EntityId player_eid = context.orbit.player_entity();
        if (const Entity *player = context.world.entities().find(player_eid))
        {
            ship_pos_world = player->get_render_physics_center_of_mass_world(alpha_f);
        }

        WorldVec3 align_delta = ship_pos_world - predicted_now;
        const double align_len = ManeuverUtil::safe_length(glm::dvec3(align_delta));
        if (!std::isfinite(align_len) || align_len > 10'000.0)
        {
            align_delta = WorldVec3(0.0, 0.0, 0.0);
        }
        return align_delta;
    }
} // namespace Game
