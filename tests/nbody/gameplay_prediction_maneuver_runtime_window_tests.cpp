#include "gameplay_prediction_maneuver_test_common.h"
#include "game/orbit/orbit_prediction_math.h"
#include "game/states/gameplay/maneuver_nbody/maneuver_prediction_bridge.h"
#include "game/states/gameplay/maneuver_nbody/maneuver_runtime_cache_builder.h"
#include "game/states/gameplay/prediction_nbody/draw/gameplay_state_prediction_draw_internal.h"

namespace
{
    void register_player_draw_subject(Game::GameplayState &state, Game::Entity &entity)
    {
        entity.set_position_world(WorldVec3(7'000'000.0, 0.0, 0.0));
        GameplayTestHooks::register_entity(&entity);

        Game::OrbiterInfo player{};
        player.entity = entity.id();
        player.is_player = true;
        state._orbit.orbiters().push_back(player);
        state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, entity.id().value};
    }

    void mark_cache_current_display_frame(Game::GameplayState &state,
                                          Game::OrbitPredictionCache &cache)
    {
        cache.display.resolved_frame_spec = orbitsim::TrajectoryFrameSpec::inertial();
        cache.display.resolved_frame_spec_valid = true;
        cache.display.display_frame_key = Game::prediction_display_frame_key(cache.display.resolved_frame_spec);
        cache.display.display_frame_revision = state.prediction_for_test().display_frame_revision;
    }

    Game::OrbitPredictionCache make_draw_ready_cache(Game::GameplayState &state,
                                                     const uint64_t generation_id,
                                                     const double t0_s,
                                                     const double t1_s)
    {
        Game::OrbitPredictionCache cache =
                make_prediction_cache(generation_id, t0_s, t1_s, 7'000'000.0, 7'400'000.0);
        mark_cache_current_display_frame(state, cache);
        return cache;
    }

    void set_planned_path(Game::OrbitPredictionCache &cache,
                          const double t0_s,
                          const double t_mid_s,
                          const double t1_s)
    {
        cache.solver.planned.trajectory_inertial = {
                make_sample(t0_s, 7'000'000.0),
                make_sample(t_mid_s, 7'200'000.0),
                make_sample(t1_s, 7'400'000.0),
        };
        cache.solver.planned.trajectory_segments_inertial = {
                make_segment(t0_s, t_mid_s, 7'000'000.0, 7'200'000.0),
                make_segment(t_mid_s, t1_s, 7'200'000.0, 7'400'000.0),
        };
        seed_planned_chunk_assembly(cache);
    }

    Game::PredictionDrawDetail::PredictionGlobalDrawContext make_draw_global_context(const double display_time_s)
    {
        Game::PredictionDrawDetail::PredictionGlobalDrawContext global_ctx{};
        global_ctx.display_time_s = display_time_s;
        global_ctx.alpha_f = 1.0f;
        global_ctx.viewport_height_px = 720.0f;
        global_ctx.tan_half_fov = 1.0;
        global_ctx.color_orbit_plan = glm::vec4(1.0f);
        return global_ctx;
    }
} // namespace

TEST(GameplayPredictionManeuverTests, ClearPredictionRuntimeResetsTrackState)
{
    Game::GameplayState state{};

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.cache.identity.valid = true;
    track.cache.identity.build_time_s = 42.0;
    track.cache.solver.base.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(60.0, 7'100'000.0)};
    track.cache.solver.base.trajectory_segments_inertial = {make_segment(0.0, 60.0, 7'000'000.0, 7'100'000.0)};
    track.request_pending = true;
    track.derived_request_pending = true;
    track.dirty = true;
    track.invalidated_while_pending = true;
    track.solver_ms_last = 12.0;
    track.solver_diagnostics.status = Game::OrbitPredictionService::Status::Success;
    track.derived_diagnostics.status = Game::PredictionDerivedStatus::Success;
    state.prediction_for_test().tracks.push_back(track);
    state.prediction_for_test().dirty = true;

    state.clear_prediction_runtime();

    ASSERT_EQ(state.prediction_for_test().tracks.size(), 1u);
    const Game::PredictionTrackState &cleared = state.prediction_for_test().tracks.front();
    EXPECT_FALSE(cleared.cache.identity.valid);
    EXPECT_TRUE(cleared.cache.solver.base.trajectory_inertial.empty());
    EXPECT_TRUE(cleared.cache.solver.base.trajectory_segments_inertial.empty());
    EXPECT_FALSE(cleared.request_pending);
    EXPECT_FALSE(cleared.derived_request_pending);
    EXPECT_EQ(cleared.latest_requested_generation_id, 0u);
    EXPECT_FALSE(cleared.dirty);
    EXPECT_FALSE(cleared.invalidated_while_pending);
    EXPECT_EQ(cleared.preview_state, Game::PredictionPreviewRuntimeState::Idle);
    EXPECT_FALSE(cleared.preview_anchor.valid);
    EXPECT_TRUE(std::isnan(cleared.preview_entered_at_s));
    EXPECT_TRUE(std::isnan(cleared.preview_last_anchor_refresh_at_s));
    EXPECT_TRUE(std::isnan(cleared.preview_last_request_at_s));
    EXPECT_DOUBLE_EQ(cleared.solver_ms_last, 0.0);
    EXPECT_EQ(cleared.solver_diagnostics.status, Game::OrbitPredictionService::Status::None);
    EXPECT_EQ(cleared.derived_diagnostics.status, Game::PredictionDerivedStatus::None);
    EXPECT_FALSE(state.prediction_for_test().dirty);
}

TEST(GameplayPredictionManeuverTests, PredictionFutureWindowClampsNegativeValues)
{
    Game::GameplayState state{};
    state.prediction_for_test().sampling_policy.orbiter_min_window_s = -5.0;
    state.prediction_for_test().sampling_policy.celestial_min_window_s = -10.0;

    const Game::PredictionSubjectKey orbiter_key{Game::PredictionSubjectKind::Orbiter, 1};
    const Game::PredictionSubjectKey celestial_key{Game::PredictionSubjectKind::Celestial, 2};

    EXPECT_DOUBLE_EQ(make_prediction_adapter(state).prediction_future_window_s(orbiter_key), 0.0);
    EXPECT_DOUBLE_EQ(make_prediction_adapter(state).prediction_future_window_s(celestial_key), 0.0);
}

TEST(GameplayPredictionManeuverTests, PredictionRequiredWindowAnchorsPlanHorizonAtFirstFutureNode)
{
    Game::GameplayState state{};
    state.prediction_for_test().draw_future_segment = true;
    state.prediction_for_test().sampling_policy.orbiter_min_window_s = 120.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 300.0;

    Game::ManeuverNode first{};
    first.id = 1;
    first.time_s = 150.0;
    state._maneuver.plan().nodes.push_back(first);

    Game::ManeuverNode second{};
    second.id = 2;
    second.time_s = 240.0;
    state._maneuver.plan().nodes.push_back(second);

    const Game::PredictionSubjectKey orbiter_key{Game::PredictionSubjectKind::Orbiter, 1};
    const double required_window_s = make_prediction_adapter(state).prediction_required_window_s(orbiter_key, 100.0, true);
    // The active solve anchors at the first future node, then adds the plan horizon and solve margin.
    EXPECT_DOUBLE_EQ(required_window_s, 650.0);
}

TEST(GameplayPredictionManeuverTests, PredictionRequiredWindowDoesNotAddLargeSolveMarginToNodeAnchoredPlanHorizon)
{
    Game::GameplayState state{};
    state.prediction_for_test().draw_future_segment = true;
    state.prediction_for_test().sampling_policy.orbiter_min_window_s = 120.0;
    state._maneuver.settings().plan_horizon.horizon_s = 600.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 600.0;

    Game::ManeuverNode node{};
    node.id = 1;
    node.time_s = 150.0;
    state._maneuver.plan().nodes.push_back(node);

    const Game::PredictionSubjectKey orbiter_key{Game::PredictionSubjectKind::Orbiter, 1};
    const double required_window_s = make_prediction_adapter(state).prediction_required_window_s(orbiter_key, 100.0, true);
    EXPECT_DOUBLE_EQ(required_window_s, 650.0);
}

TEST(GameplayPredictionManeuverTests, FullRequestAnchorsPlanHorizonAfterNearbyNode)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state.prediction_for_test().draw_future_segment = true;
    state.prediction_for_test().sampling_policy.orbiter_min_window_s = 120.0;
    state._maneuver.settings().plan_horizon.horizon_s = 600.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 600.0;

    Game::ManeuverNode node{};
    node.id = 1;
    node.time_s = 150.0;
    state._maneuver.plan().selected_node_id = node.id;
    state._maneuver.plan().nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    EXPECT_EQ(request.options.solve_quality, Game::OrbitPredictionService::SolveQuality::Full);
    EXPECT_DOUBLE_EQ(request.options.future_window_s, 650.0);
    ASSERT_EQ(request.maneuver.maneuver_impulses.size(), 1u);
    EXPECT_EQ(request.maneuver.maneuver_impulses.front().node_id, node.id);
    EXPECT_DOUBLE_EQ(request.maneuver.maneuver_impulses.front().t_s, node.time_s);
}

TEST(GameplayPredictionManeuverTests, FullRequestKeepsFarFutureNodeWhenPlanHorizonIsShorterThanNodeTime)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state.prediction_for_test().draw_future_segment = true;
    state.prediction_for_test().sampling_policy.orbiter_min_window_s = 120.0;
    state._maneuver.settings().plan_horizon.horizon_s = 600.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 300.0;

    Game::ManeuverNode node{};
    node.id = 1;
    node.time_s = 50'000.0;
    state._maneuver.plan().selected_node_id = node.id;
    state._maneuver.plan().nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    EXPECT_EQ(request.options.solve_quality, Game::OrbitPredictionService::SolveQuality::Full);
    EXPECT_DOUBLE_EQ(request.options.future_window_s, 50'500.0);
    ASSERT_EQ(request.maneuver.maneuver_impulses.size(), 1u);
    EXPECT_EQ(request.maneuver.maneuver_impulses.front().node_id, node.id);
    EXPECT_DOUBLE_EQ(request.maneuver.maneuver_impulses.front().t_s, node.time_s);
}

TEST(GameplayPredictionManeuverTests, FullRequestUsesConfiguredPlanHorizonForNearFutureNode)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(23.8);
    ASSERT_TRUE(state._orbit.scenario_owner());
    state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state.prediction_for_test().draw_future_segment = true;
    state.prediction_for_test().sampling_policy.orbiter_min_window_s = 600.0;
    state._maneuver.settings().plan_horizon.horizon_s = 60'000.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 60'000.0;

    Game::ManeuverNode node{};
    node.id = 1;
    node.time_s = 372.248;
    state._maneuver.plan().selected_node_id = node.id;
    state._maneuver.plan().nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              23.8,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    EXPECT_EQ(request.options.solve_quality, Game::OrbitPredictionService::SolveQuality::Full);
    EXPECT_DOUBLE_EQ(request.options.future_window_s, 60'348.448);
    ASSERT_EQ(request.maneuver.maneuver_impulses.size(), 1u);
    EXPECT_EQ(request.maneuver.maneuver_impulses.front().node_id, node.id);
    EXPECT_DOUBLE_EQ(request.maneuver.maneuver_impulses.front().t_s, node.time_s);
}

TEST(GameplayPredictionManeuverTests, ShortCurrentCacheRebuildsToConfiguredPlanHorizon)
{
    Game::GameplayState state{};
    state.prediction_for_test().draw_future_segment = true;
    state.prediction_for_test().sampling_policy.orbiter_min_window_s = 600.0;
    state._maneuver.settings().plan_horizon.horizon_s = 60'000.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 60'000.0;

    Game::ManeuverNode node{};
    node.id = 1;
    node.time_s = 372.248;
    state._maneuver.plan().selected_node_id = node.id;
    state._maneuver.plan().nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.supports_maneuvers = true;
    track.cache = make_prediction_cache(17u, 23.8, 623.8, 7'000'000.0, 7'300'000.0);

    const double required_window_s = make_prediction_adapter(state).prediction_required_window_s(track, 23.8, true);
    EXPECT_DOUBLE_EQ(required_window_s, 60'348.448);
    EXPECT_TRUE(make_prediction_adapter(state).should_rebuild_prediction_track(track, 23.8, 1.0f / 60.0f, false, true));
}

TEST(GameplayPredictionManeuverTests, PredictionRequiredWindowExtendsPastFarFutureManeuverNode)
{
    Game::GameplayState state{};
    state.prediction_for_test().draw_future_segment = true;
    state.prediction_for_test().sampling_policy.orbiter_min_window_s = 120.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 300.0;

    Game::ManeuverNode far_node{};
    far_node.id = 1;
    far_node.time_s = 50'000.0;
    state._maneuver.plan().nodes.push_back(far_node);

    const Game::PredictionSubjectKey orbiter_key{Game::PredictionSubjectKind::Orbiter, 1};
    const double required_window_s = make_prediction_adapter(state).prediction_required_window_s(orbiter_key, 100.0, true);
    EXPECT_DOUBLE_EQ(required_window_s, 50'500.0);
}

TEST(GameplayPredictionManeuverTests, PredictionRequiredWindowUsesNodeAnchoredPlanHorizonBeyondBaseSamplingWindow)
{
    Game::GameplayState state{};
    state.prediction_for_test().draw_future_segment = true;
    state.prediction_for_test().sampling_policy.orbiter_min_window_s = 60'000.0;
    state._maneuver.settings().plan_horizon.horizon_s = 120'000.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 300.0;

    Game::ManeuverNode node{};
    node.id = 1;
    node.time_s = 1'000.0;
    state._maneuver.plan().nodes.push_back(node);

    const Game::PredictionSubjectKey orbiter_key{Game::PredictionSubjectKind::Orbiter, 1};
    const double required_window_s = make_prediction_adapter(state).prediction_required_window_s(orbiter_key, 0.0, true);
    EXPECT_DOUBLE_EQ(required_window_s, 121'000.0);
}

TEST(GameplayPredictionManeuverTests, PredictionRequiredWindowKeepsDisplayedHorizonDuringLivePreview)
{
    Game::GameplayState state{};
    state.prediction_for_test().draw_future_segment = true;
    state.prediction_for_test().sampling_policy.orbiter_min_window_s = 50'000.0;
    state._maneuver.settings().plan_windows.preview_window_s = 180.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 300.0;
    state._maneuver.settings().live_preview_active = true;

    Game::ManeuverNode selected{};
    selected.id = 7;
    selected.time_s = 240.0;
    state._maneuver.plan().selected_node_id = selected.id;
    state._maneuver.plan().nodes.push_back(selected);

    const Game::PredictionSubjectKey orbiter_key{Game::PredictionSubjectKind::Orbiter, 1};
    const double required_window_s = make_prediction_adapter(state).prediction_required_window_s(orbiter_key, 100.0, true);
    EXPECT_DOUBLE_EQ(required_window_s, 50'000.0);
}

TEST(GameplayPredictionManeuverTests, PredictionRequiredWindowUsesConfiguredPlanHorizonForNearFutureNode)
{
    Game::GameplayState state{};
    state.prediction_for_test().draw_future_segment = true;
    state.prediction_for_test().sampling_policy.orbiter_min_window_s = 600.0;
    state._maneuver.settings().plan_horizon.horizon_s = 60'000.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 60'000.0;

    Game::ManeuverNode node{};
    node.id = 1;
    node.time_s = 372.248;
    state._maneuver.plan().selected_node_id = node.id;
    state._maneuver.plan().nodes.push_back(node);

    const Game::PredictionSubjectKey orbiter_key{Game::PredictionSubjectKind::Orbiter, 1};
    const double required_window_s = make_prediction_adapter(state).prediction_required_window_s(orbiter_key, 23.8, true);
    EXPECT_DOUBLE_EQ(required_window_s, 60'348.448);
}

TEST(GameplayPredictionManeuverTests, PreviewAnchorCacheSeparatesPatchWindowFromDisplayWindow)
{
    Game::GameplayState state{};
    state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state.prediction_for_test().draw_future_segment = true;
    state.prediction_for_test().sampling_policy.orbiter_min_window_s = 120.0;
    state._maneuver.settings().plan_windows.preview_window_s = 180.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 300.0;
    state._maneuver.settings().live_preview_active = true;
    state._maneuver.gizmo_interaction().state = Game::ManeuverGizmoInteraction::State::DragAxis;

    Game::ManeuverNode far_node{};
    far_node.id = 7;
    far_node.time_s = 50'000.0;
    state._maneuver.plan().selected_node_id = far_node.id;
    state._maneuver.plan().nodes.push_back(far_node);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;
    track.cache.identity.generation_id = 42;

    make_prediction_adapter(state).refresh_prediction_preview_anchor(track, 100.0, true);

    ASSERT_TRUE(track.preview_anchor.valid);
    EXPECT_EQ(track.preview_state, Game::PredictionPreviewRuntimeState::EnterDrag);
    EXPECT_EQ(track.preview_anchor.anchor_node_id, far_node.id);
    EXPECT_DOUBLE_EQ(track.preview_anchor.anchor_time_s, 50'000.0);
    EXPECT_DOUBLE_EQ(make_prediction_adapter(state).prediction_display_window_s(track.key, 100.0, true), 180.0);
    EXPECT_DOUBLE_EQ(make_prediction_adapter(state).prediction_preview_exact_window_s(track, 100.0, true), 180.0);
    EXPECT_DOUBLE_EQ(track.preview_anchor.request_window_s, 50'500.0);
    EXPECT_DOUBLE_EQ(track.preview_anchor.visual_window_s, 180.0);
    EXPECT_DOUBLE_EQ(track.preview_anchor.exact_window_s, 300.0);
    EXPECT_DOUBLE_EQ(track.preview_last_anchor_refresh_at_s, 100.0);
}

TEST(GameplayPredictionManeuverTests, PreviewAnchorCacheClampsVisualWindowToPreviewWindowDuringDrag)
{
    Game::GameplayState state{};
    state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state.prediction_for_test().draw_future_segment = true;
    state.prediction_for_test().sampling_policy.orbiter_min_window_s = 2.0 * Game::OrbitPredictionTuning::kSecondsPerDay;
    state._maneuver.settings().plan_windows.preview_window_s = 180.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 300.0;
    state._maneuver.settings().live_preview_active = true;
    state._maneuver.gizmo_interaction().state = Game::ManeuverGizmoInteraction::State::DragAxis;

    Game::ManeuverNode node{};
    node.id = 9;
    node.time_s = 240.0;
    state._maneuver.plan().selected_node_id = node.id;
    state._maneuver.plan().nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;

    make_prediction_adapter(state).refresh_prediction_preview_anchor(track, 100.0, true);

    ASSERT_TRUE(track.preview_anchor.valid);
    EXPECT_DOUBLE_EQ(track.preview_anchor.visual_window_s, 180.0);
    EXPECT_DOUBLE_EQ(track.preview_anchor.exact_window_s, 300.0);
    EXPECT_LT(track.preview_anchor.visual_window_s, track.preview_anchor.exact_window_s);
    EXPECT_DOUBLE_EQ(track.preview_anchor.request_window_s,
                     2.0 * Game::OrbitPredictionTuning::kSecondsPerDay);
}

TEST(GameplayPredictionManeuverTests, PreviewAnchorCacheCanReenterDragFromStableIdleTrack)
{
    Game::GameplayState state{};
    state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._maneuver.settings().plan_windows.preview_window_s = 180.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 300.0;
    state._maneuver.settings().live_preview_active = true;
    state._maneuver.gizmo_interaction().state = Game::ManeuverGizmoInteraction::State::DragAxis;

    Game::ManeuverNode node{};
    node.id = 11;
    node.time_s = 240.0;
    state._maneuver.plan().selected_node_id = node.id;
    state._maneuver.plan().nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;
    track.cache = make_prediction_cache(4u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.authoritative_cache = track.cache;
    track.dirty = false;
    track.preview_state = Game::PredictionPreviewRuntimeState::Idle;

    make_prediction_adapter(state).refresh_prediction_preview_anchor(track, 100.0, true);

    EXPECT_TRUE(track.preview_anchor.valid);
    EXPECT_EQ(track.preview_state, Game::PredictionPreviewRuntimeState::EnterDrag);
}

TEST(GameplayPredictionManeuverTests, PreviewAnchorCacheTransitionsToAwaitFullRefineAfterDragEnds)
{
    Game::GameplayState state{};
    state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._maneuver.settings().plan_windows.preview_window_s = 180.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 300.0;
    state._maneuver.settings().live_preview_active = true;
    state._maneuver.gizmo_interaction().state = Game::ManeuverGizmoInteraction::State::DragAxis;

    Game::ManeuverNode node{};
    node.id = 3;
    node.time_s = 240.0;
    state._maneuver.plan().selected_node_id = node.id;
    state._maneuver.plan().nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;

    make_prediction_adapter(state).refresh_prediction_preview_anchor(track, 100.0, true);
    ASSERT_EQ(track.preview_state, Game::PredictionPreviewRuntimeState::EnterDrag);

    track.preview_state = Game::PredictionPreviewRuntimeState::PreviewStreaming;
    state._maneuver.settings().live_preview_active = false;
    state._maneuver.gizmo_interaction().state = Game::ManeuverGizmoInteraction::State::Idle;
    make_prediction_adapter(state).refresh_prediction_preview_anchor(track, 101.0, true);

    EXPECT_EQ(track.preview_state, Game::PredictionPreviewRuntimeState::AwaitFullRefine);
    EXPECT_TRUE(track.preview_anchor.valid);
    EXPECT_EQ(track.preview_anchor.anchor_node_id, node.id);
}

TEST(GameplayPredictionManeuverTests, RequestOrbiterPredictionTracksPreviewRequestTimestamp)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._maneuver.settings().plan_windows.preview_window_s = 180.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 300.0;
    state._maneuver.settings().live_preview_active = true;
    state._maneuver.gizmo_interaction().state = Game::ManeuverGizmoInteraction::State::DragAxis;

    Game::ManeuverNode node{};
    node.id = 9;
    node.time_s = 240.0;
    state._maneuver.plan().selected_node_id = node.id;
    state._maneuver.plan().nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;
    make_prediction_adapter(state).refresh_prediction_preview_anchor(track, 100.0, true);

    const bool requested = make_prediction_adapter(state).request_orbiter_prediction_async(track,
                                                                  WorldVec3(7'000'000.0, 0.0, 0.0),
                                                                  glm::dvec3(0.0, 7'500.0, 0.0),
                                                                  100.0,
                                                                  false,
                                                                  true);

    ASSERT_TRUE(requested);
    EXPECT_TRUE(track.request_pending);
    EXPECT_EQ(track.pending_solve_quality, Game::OrbitPredictionService::SolveQuality::FastPreview);
    EXPECT_EQ(track.preview_state, Game::PredictionPreviewRuntimeState::DragPreviewPending);
    EXPECT_DOUBLE_EQ(track.preview_last_request_at_s, 100.0);
    EXPECT_GT(track.latest_requested_generation_id, 0u);
}

TEST(GameplayPredictionManeuverTests, FastPreviewRequestKeepsUpstreamManeuversBeforeAnchor)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._maneuver.settings().plan_windows.preview_window_s = 180.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 300.0;
    state._maneuver.settings().live_preview_active = true;
    state._maneuver.gizmo_interaction().state = Game::ManeuverGizmoInteraction::State::DragAxis;

    Game::ManeuverNode first{};
    first.id = 1;
    first.time_s = 150.0;
    state._maneuver.plan().nodes.push_back(first);

    Game::ManeuverNode second{};
    second.id = 2;
    second.time_s = 240.0;
    state._maneuver.plan().selected_node_id = second.id;
    state._maneuver.plan().nodes.push_back(second);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;

    Game::OrbitPredictionService::Request queued{};
    const bool requested = build_orbiter_prediction_request(state, track,
                                                                  WorldVec3(7'000'000.0, 0.0, 0.0),
                                                                  glm::dvec3(0.0, 7'500.0, 0.0),
                                                                  100.0,
                                                                  false,
                                                                  true,
                                                                  queued);

    ASSERT_TRUE(requested);
    ASSERT_EQ(queued.options.solve_quality, Game::OrbitPredictionService::SolveQuality::FastPreview);
    ASSERT_EQ(queued.maneuver.maneuver_impulses.size(), 2u);
    EXPECT_EQ(queued.maneuver.maneuver_impulses[0].node_id, first.id);
    EXPECT_EQ(queued.maneuver.maneuver_impulses[1].node_id, second.id);
}

TEST(GameplayPredictionManeuverTests, FastPreviewRequestKeepsDownstreamManeuversBeyondPreviewDisplayWindow)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state.prediction_for_test().draw_future_segment = true;
    state.prediction_for_test().sampling_policy.orbiter_min_window_s = 120.0;
    state._maneuver.settings().plan_windows.preview_window_s = 180.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 300.0;
    state._maneuver.settings().live_preview_active = true;
    state._maneuver.gizmo_interaction().state = Game::ManeuverGizmoInteraction::State::DragAxis;

    Game::ManeuverNode anchor{};
    anchor.id = 1;
    anchor.time_s = 240.0;
    state._maneuver.plan().selected_node_id = anchor.id;
    state._maneuver.plan().nodes.push_back(anchor);

    Game::ManeuverNode downstream{};
    downstream.id = 2;
    downstream.time_s = 700.0; // Outside the local patch window, still inside the full request horizon.
    state._maneuver.plan().nodes.push_back(downstream);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;

    Game::OrbitPredictionService::Request queued{};
    const bool requested = build_orbiter_prediction_request(state, track,
                                                                  WorldVec3(7'000'000.0, 0.0, 0.0),
                                                                  glm::dvec3(0.0, 7'500.0, 0.0),
                                                                  100.0,
                                                                  false,
                                                                  true,
                                                                  queued);

    ASSERT_TRUE(requested);
    ASSERT_EQ(queued.options.solve_quality, Game::OrbitPredictionService::SolveQuality::FastPreview);
    ASSERT_EQ(queued.maneuver.maneuver_impulses.size(), 2u);
    EXPECT_EQ(queued.maneuver.maneuver_impulses[0].node_id, anchor.id);
    EXPECT_EQ(queued.maneuver.maneuver_impulses[1].node_id, downstream.id);
}

TEST(GameplayPredictionManeuverTests, FastPreviewRequestCapsHorizonToExactPatchWindow)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state.prediction_for_test().draw_future_segment = true;
    state.prediction_for_test().sampling_policy.orbiter_min_window_s =
            2.0 * Game::OrbitPredictionTuning::kSecondsPerDay;
    state._maneuver.settings().plan_windows.preview_window_s = 180.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 300.0;
    state._maneuver.settings().live_preview_active = true;
    state._maneuver.gizmo_interaction().state = Game::ManeuverGizmoInteraction::State::DragAxis;

    Game::ManeuverNode selected{};
    selected.id = 4;
    selected.time_s = 240.0;
    state._maneuver.plan().selected_node_id = selected.id;
    state._maneuver.plan().nodes.push_back(selected);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    ASSERT_EQ(request.options.solve_quality, Game::OrbitPredictionService::SolveQuality::FastPreview);
    ASSERT_TRUE(request.maneuver.preview_patch.active);
    EXPECT_DOUBLE_EQ(request.options.future_window_s, 500.0);
    EXPECT_DOUBLE_EQ(request.maneuver.preview_patch.anchor_time_s, selected.time_s);
    EXPECT_DOUBLE_EQ(request.maneuver.preview_patch.visual_window_s, 180.0);
    EXPECT_DOUBLE_EQ(request.maneuver.preview_patch.exact_window_s, 180.0);
    EXPECT_GT(request.options.future_window_s, request.maneuver.preview_patch.visual_window_s);
    EXPECT_LT(request.options.future_window_s, 2.0 * Game::OrbitPredictionTuning::kSecondsPerDay);
}

TEST(GameplayPredictionManeuverTests, FastPreviewRequestUsesSelectedNodePreviewForAnchorState)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._maneuver.settings().plan_windows.preview_window_s = 180.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 300.0;
    state._maneuver.settings().live_preview_active = true;
    state._maneuver.gizmo_interaction().state = Game::ManeuverGizmoInteraction::State::DragAxis;

    Game::ManeuverNode upstream{};
    upstream.id = 1;
    upstream.time_s = 150.0;
    state._maneuver.plan().nodes.push_back(upstream);

    Game::ManeuverNode selected{};
    selected.id = 2;
    selected.time_s = 240.0;
    state._maneuver.plan().selected_node_id = selected.id;
    state._maneuver.plan().nodes.push_back(selected);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;
    track.cache.solver.base.trajectory_inertial = {
            make_sample(100.0, 7'000'000.0),
            make_sample(240.0, 7'100'000.0),
    };
    track.cache.solver.planned.maneuver_previews.push_back(Game::OrbitPredictionService::ManeuverNodePreview{
            .node_id = selected.id,
            .t_s = selected.time_s,
            .valid = true,
            .inertial_position_m = glm::dvec3(7'250'000.0, 123.0, 0.0),
            .inertial_velocity_mps = glm::dvec3(0.0, 7'650.0, 5.0),
    });

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    ASSERT_EQ(request.options.solve_quality, Game::OrbitPredictionService::SolveQuality::FastPreview);
    ASSERT_TRUE(request.maneuver.preview_patch.active);
    EXPECT_FALSE(request.maneuver.planned_suffix_refine.active);
    ASSERT_TRUE(request.maneuver.preview_patch.anchor_state_valid);
    EXPECT_TRUE(request.maneuver.preview_patch.anchor_state_trusted);
    EXPECT_DOUBLE_EQ(request.maneuver.preview_patch.anchor_state_inertial.position_m.x, 7'250'000.0);
    EXPECT_DOUBLE_EQ(request.maneuver.preview_patch.anchor_state_inertial.position_m.y, 123.0);
    EXPECT_DOUBLE_EQ(request.maneuver.preview_patch.anchor_state_inertial.velocity_mps.y, 7'650.0);
    EXPECT_DOUBLE_EQ(request.maneuver.preview_patch.anchor_state_inertial.velocity_mps.z, 5.0);
}

TEST(GameplayPredictionManeuverTests, FullRequestEnablesPlannedSuffixRefineForPostDragAnchor)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};

    Game::ManeuverNode upstream{};
    upstream.id = 1;
    upstream.time_s = 150.0;
    state._maneuver.plan().nodes.push_back(upstream);

    Game::ManeuverNode selected{};
    selected.id = 2;
    selected.time_s = 240.0;
    state._maneuver.plan().selected_node_id = selected.id;
    state._maneuver.plan().nodes.push_back(selected);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;
    track.cache = make_prediction_cache(4u, 100.0, 360.0, 7'000'000.0, 7'260'000.0);
    track.cache.solver.planned.trajectory_segments_inertial = {
            make_segment(100.0, 150.0, 7'000'000.0, 7'050'000.0),
            make_segment(150.0, 240.0, 7'050'000.0, 7'140'000.0),
    };
    track.cache.solver.planned.trajectory_inertial = {
            make_sample(100.0, 7'000'000.0),
            make_sample(150.0, 7'050'000.0),
            make_sample(240.0, 7'140'000.0),
    };
    track.cache.solver.planned.maneuver_previews.push_back(Game::OrbitPredictionService::ManeuverNodePreview{
            .node_id = upstream.id,
            .t_s = upstream.time_s,
            .valid = true,
            .inertial_position_m = glm::dvec3(7'050'000.0, 0.0, 0.0),
            .inertial_velocity_mps = glm::dvec3(0.0, 7'500.0, 0.0),
    });
    track.cache.solver.planned.maneuver_previews.push_back(Game::OrbitPredictionService::ManeuverNodePreview{
            .node_id = selected.id,
            .t_s = selected.time_s,
            .valid = true,
            .inertial_position_m = glm::dvec3(7'140'000.0, 0.0, 0.0),
            .inertial_velocity_mps = glm::dvec3(0.0, 7'500.0, 0.0),
    });
    track.authoritative_cache = track.cache;
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = selected.id;
    track.preview_anchor.anchor_time_s = selected.time_s;

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    EXPECT_EQ(request.options.solve_quality, Game::OrbitPredictionService::SolveQuality::Full);
    ASSERT_TRUE(request.maneuver.planned_suffix_refine.active);
    EXPECT_EQ(request.maneuver.planned_suffix_refine.anchor_node_id, selected.id);
    EXPECT_DOUBLE_EQ(request.maneuver.planned_suffix_refine.anchor_time_s, selected.time_s);
    ASSERT_EQ(request.maneuver.planned_suffix_refine.prefix_segments_inertial.size(), 2u);
    EXPECT_DOUBLE_EQ(request.maneuver.planned_suffix_refine.prefix_segments_inertial.front().t0_s, 100.0);
    EXPECT_DOUBLE_EQ(request.maneuver.planned_suffix_refine.prefix_segments_inertial.back().t0_s +
                             request.maneuver.planned_suffix_refine.prefix_segments_inertial.back().dt_s,
                     selected.time_s);
    ASSERT_EQ(request.maneuver.planned_suffix_refine.prefix_previews.size(), 1u);
    EXPECT_EQ(request.maneuver.planned_suffix_refine.prefix_previews.front().node_id, upstream.id);
    EXPECT_FALSE(request.maneuver.preview_patch.active);
}

TEST(GameplayPredictionManeuverTests, FastPreviewRequestFallsBackToInertialCacheForAnchorState)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._maneuver.settings().plan_windows.preview_window_s = 180.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 300.0;
    state._maneuver.settings().live_preview_active = true;
    state._maneuver.gizmo_interaction().state = Game::ManeuverGizmoInteraction::State::DragAxis;

    Game::ManeuverNode selected{};
    selected.id = 5;
    selected.time_s = 240.0;
    state._maneuver.plan().selected_node_id = selected.id;
    state._maneuver.plan().nodes.push_back(selected);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;
    track.cache.solver.base.trajectory_inertial = {
            make_sample(100.0, 7'000'000.0),
            make_sample(240.0, 7'100'000.0),
    };

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    ASSERT_EQ(request.options.solve_quality, Game::OrbitPredictionService::SolveQuality::FastPreview);
    ASSERT_TRUE(request.maneuver.preview_patch.active);
    ASSERT_TRUE(request.maneuver.preview_patch.anchor_state_valid);
    EXPECT_TRUE(request.maneuver.preview_patch.anchor_state_trusted);
    EXPECT_DOUBLE_EQ(request.maneuver.preview_patch.anchor_state_inertial.position_m.x, 7'100'000.0);
    EXPECT_DOUBLE_EQ(request.maneuver.preview_patch.anchor_state_inertial.velocity_mps.y, 7'500.0);
}

TEST(GameplayPredictionManeuverTests, TimeEditActivatesFastPreviewRequest)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._maneuver.settings().plan_windows.preview_window_s = 180.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 300.0;
    state._maneuver.settings().live_preview_active = true;

    Game::ManeuverNode selected{};
    selected.id = 5;
    selected.time_s = 360.0;
    selected.dv_rtn_mps = glm::dvec3(0.0, 10.0, 0.0);
    state._maneuver.plan().selected_node_id = selected.id;
    state._maneuver.plan().nodes.push_back(selected);
    state._maneuver.edit_preview().state = Game::ManeuverNodeEditPreview::State::EditingTime;
    state._maneuver.edit_preview().node_id = selected.id;
    state._maneuver.edit_preview().changed = true;
    state._maneuver.edit_preview().start_time_s = 240.0;

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;
    track.cache.solver.base.trajectory_inertial = {
            make_sample(100.0, 7'000'000.0),
            make_sample(selected.time_s, 7'200'000.0),
    };

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    EXPECT_EQ(request.options.solve_quality, Game::OrbitPredictionService::SolveQuality::FastPreview);
    ASSERT_TRUE(request.maneuver.preview_patch.active);
    EXPECT_DOUBLE_EQ(request.maneuver.preview_patch.anchor_time_s, selected.time_s);
    ASSERT_EQ(request.maneuver.maneuver_impulses.size(), 1u);
    EXPECT_DOUBLE_EQ(request.maneuver.maneuver_impulses.front().t_s, selected.time_s);
}

TEST(GameplayPredictionManeuverTests, TimeEditUsesBaselineAnchorStateForMovedFirstNode)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._maneuver.settings().plan_windows.preview_window_s = 180.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 300.0;
    state._maneuver.settings().live_preview_active = true;

    Game::ManeuverNode selected{};
    selected.id = 5;
    selected.time_s = 300.0;
    selected.dv_rtn_mps = glm::dvec3(0.0, 10.0, 0.0);
    state._maneuver.plan().selected_node_id = selected.id;
    state._maneuver.plan().nodes.push_back(selected);
    state._maneuver.edit_preview().state = Game::ManeuverNodeEditPreview::State::EditingTime;
    state._maneuver.edit_preview().node_id = selected.id;
    state._maneuver.edit_preview().changed = true;
    state._maneuver.edit_preview().start_time_s = 240.0;

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;
    track.cache.identity.valid = true;
    track.cache.solver.base.trajectory_inertial = {
            make_sample(100.0, 7'000'000.0),
            make_sample(selected.time_s, 7'200'000.0),
    };
    track.cache.solver.planned.trajectory_inertial = {
            make_sample(100.0, 7'000'000.0),
            make_sample(selected.time_s, 9'000'000.0),
    };
    track.cache.solver.planned.maneuver_previews.push_back(Game::OrbitPredictionService::ManeuverNodePreview{
            .node_id = selected.id,
            .t_s = selected.time_s,
            .valid = true,
            .inertial_position_m = glm::dvec3(9'000'000.0, 0.0, 0.0),
            .inertial_velocity_mps = glm::dvec3(0.0, 7'500.0, 0.0),
    });

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    ASSERT_EQ(request.options.solve_quality, Game::OrbitPredictionService::SolveQuality::FastPreview);
    ASSERT_TRUE(request.maneuver.preview_patch.active);
    ASSERT_TRUE(request.maneuver.preview_patch.anchor_state_valid);
    EXPECT_TRUE(request.maneuver.preview_patch.anchor_state_trusted);
    EXPECT_DOUBLE_EQ(request.maneuver.preview_patch.anchor_state_inertial.position_m.x, 7'200'000.0);
}

TEST(GameplayPredictionManeuverTests, TimeEditLeavesAnchorStateUnseededWhenPriorManeuverExists)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state._maneuver.settings().plan_windows.preview_window_s = 180.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 300.0;
    state._maneuver.settings().live_preview_active = true;

    Game::ManeuverNode upstream{};
    upstream.id = 4;
    upstream.time_s = 180.0;
    upstream.dv_rtn_mps = glm::dvec3(0.0, 5.0, 0.0);

    Game::ManeuverNode selected{};
    selected.id = 5;
    selected.time_s = 300.0;
    selected.dv_rtn_mps = glm::dvec3(0.0, 10.0, 0.0);
    state._maneuver.plan().selected_node_id = selected.id;
    state._maneuver.plan().nodes.push_back(upstream);
    state._maneuver.plan().nodes.push_back(selected);
    state._maneuver.edit_preview().state = Game::ManeuverNodeEditPreview::State::EditingTime;
    state._maneuver.edit_preview().node_id = selected.id;
    state._maneuver.edit_preview().changed = true;
    state._maneuver.edit_preview().start_time_s = 240.0;

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;
    track.cache.solver.base.trajectory_inertial = {
            make_sample(100.0, 7'000'000.0),
            make_sample(selected.time_s, 7'200'000.0),
    };

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    ASSERT_EQ(request.options.solve_quality, Game::OrbitPredictionService::SolveQuality::FastPreview);
    ASSERT_TRUE(request.maneuver.preview_patch.active);
    EXPECT_FALSE(request.maneuver.preview_patch.anchor_state_valid);
    ASSERT_EQ(request.maneuver.maneuver_impulses.size(), 2u);
}

TEST(GameplayPredictionManeuverTests, TimeEditFinishRequestsFullRefine)
{
    Game::GameplayState state{};
    state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};
    state.prediction_for_test().dirty = false;

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.preview_state = Game::PredictionPreviewRuntimeState::PreviewStreaming;
    state.prediction_for_test().tracks.push_back(track);

    state._maneuver.edit_preview().state = Game::ManeuverNodeEditPreview::State::EditingTime;
    state._maneuver.edit_preview().node_id = 5;
    state._maneuver.edit_preview().changed = true;
    state._maneuver.edit_preview().start_time_s = 240.0;

    auto maneuver_prediction = make_maneuver_prediction_context(state);
    Game::ManeuverPredictionBridge::finish_node_time_edit_preview(maneuver_prediction, false);

    ASSERT_EQ(state.prediction_for_test().tracks.size(), 1u);
    EXPECT_EQ(state.prediction_for_test().tracks.front().preview_state, Game::PredictionPreviewRuntimeState::AwaitFullRefine);
    EXPECT_EQ(state._maneuver.edit_preview().state, Game::ManeuverNodeEditPreview::State::Idle);
    EXPECT_TRUE(state.prediction_for_test().dirty);
}

TEST(GameplayPredictionManeuverTests, FinishedTimeEditDoesNotSeedAnchorFromStalePlannedPath)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};

    Game::ManeuverNode selected{};
    selected.id = 5;
    selected.time_s = 300.0;
    selected.dv_rtn_mps = glm::dvec3(0.0, 10.0, 0.0);
    state._maneuver.plan().selected_node_id = selected.id;
    state._maneuver.plan().nodes.push_back(selected);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = selected.id;
    track.preview_anchor.anchor_time_s = selected.time_s;
    track.cache.identity.valid = true;
    track.cache.solver.base.trajectory_inertial = {
            make_sample(100.0, 7'000'000.0),
            make_sample(selected.time_s, 7'200'000.0),
    };
    track.cache.solver.base.trajectory_segments_inertial = {
            make_segment(100.0, selected.time_s, 7'000'000.0, 7'200'000.0),
    };
    track.cache.solver.planned.trajectory_inertial = {
            make_sample(100.0, 7'000'000.0),
            make_sample(240.0, 9'000'000.0),
            make_sample(selected.time_s, 9'500'000.0),
    };
    track.cache.solver.planned.trajectory_segments_inertial = {
            make_segment(100.0, 240.0, 7'000'000.0, 9'000'000.0),
            make_segment(240.0, selected.time_s, 9'000'000.0, 9'500'000.0),
    };
    track.cache.solver.planned.maneuver_previews.push_back(Game::OrbitPredictionService::ManeuverNodePreview{
            .node_id = selected.id,
            .t_s = 240.0,
            .valid = true,
            .inertial_position_m = glm::dvec3(9'000'000.0, 0.0, 0.0),
            .inertial_velocity_mps = glm::dvec3(0.0, 7'500.0, 0.0),
    });
    track.authoritative_cache = track.cache;

    orbitsim::State anchor_state{};
    ASSERT_TRUE(resolve_prediction_preview_anchor_state(state, track, anchor_state));
    EXPECT_DOUBLE_EQ(anchor_state.position_m.x, 7'200'000.0);

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    EXPECT_EQ(request.options.solve_quality, Game::OrbitPredictionService::SolveQuality::Full);
    EXPECT_FALSE(request.maneuver.planned_suffix_refine.active);
}

TEST(GameplayPredictionManeuverTests, DrawContextUsesAuthoritativePlannedPrefixDuringDeltaVEdit)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    Game::Entity player_entity{Game::EntityId{1}, "player"};
    register_player_draw_subject(state, player_entity);

    Game::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    node.dv_rtn_mps = glm::dvec3(0.0, 5.0, 0.0);
    state._maneuver.plan().selected_node_id = node.id;
    state._maneuver.plan().nodes.push_back(node);
    const uint64_t old_plan_signature = make_prediction_adapter(state).current_maneuver_plan_signature();
    state._maneuver.plan().nodes.front().dv_rtn_mps = glm::dvec3(0.0, 12.0, 0.0);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::PreviewStreaming;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = node.id;
    track.preview_anchor.anchor_time_s = node.time_s;
    track.preview_anchor.visual_window_s = 120.0;
    track.preview_anchor.exact_window_s = 60.0;
    state._maneuver.edit_preview().state = Game::ManeuverNodeEditPreview::State::EditingDv;
    state._maneuver.edit_preview().node_id = node.id;

    track.cache = make_draw_ready_cache(state, 5u, 100.0, 500.0);
    set_planned_path(track.cache, node.time_s, 320.0, 500.0);
    track.cache.identity.maneuver_plan_signature_valid = true;
    track.cache.identity.maneuver_plan_signature = old_plan_signature;

    track.authoritative_cache = make_draw_ready_cache(state, 4u, 100.0, 500.0);
    set_planned_path(track.authoritative_cache, 100.0, node.time_s, 500.0);
    track.authoritative_cache.identity.maneuver_plan_signature_valid = true;
    track.authoritative_cache.identity.maneuver_plan_signature = old_plan_signature;

    Game::PredictionDrawDetail::PredictionTrackDrawContext draw_ctx{};
    ASSERT_TRUE(make_prediction_adapter(state).build_orbit_prediction_track_draw_context(
            track,
            make_draw_global_context(100.0),
            draw_ctx));

    EXPECT_FALSE(draw_ctx.planned_cache_current);
    EXPECT_FALSE(draw_ctx.planned_cache_drawable);
    EXPECT_EQ(draw_ctx.stale_planned_cache, &track.authoritative_cache);
    EXPECT_TRUE(draw_ctx.stale_planned_cache_drawable);
    EXPECT_DOUBLE_EQ(draw_ctx.stale_planned_cache_prefix_cutoff_s, node.time_s);
    EXPECT_EQ(draw_ctx.planned_window_assembly,
              &track.authoritative_cache.display.planned_chunk_assembly);

    GameplayTestHooks::clear_entities();
}

TEST(GameplayPredictionManeuverTests, DrawContextLimitsTimeEditStalePrefixToEarlierNodeTime)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    Game::Entity player_entity{Game::EntityId{1}, "player"};
    register_player_draw_subject(state, player_entity);

    Game::ManeuverNode node{};
    node.id = 9;
    node.time_s = 240.0;
    state._maneuver.plan().selected_node_id = node.id;
    state._maneuver.plan().nodes.push_back(node);
    const uint64_t old_plan_signature = make_prediction_adapter(state).current_maneuver_plan_signature();
    state._maneuver.plan().nodes.front().time_s = 300.0;

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::PreviewStreaming;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = node.id;
    track.preview_anchor.anchor_time_s = 300.0;
    track.preview_anchor.visual_window_s = 120.0;
    track.preview_anchor.exact_window_s = 60.0;
    state._maneuver.edit_preview().state = Game::ManeuverNodeEditPreview::State::EditingTime;
    state._maneuver.edit_preview().node_id = node.id;
    state._maneuver.edit_preview().changed = true;
    state._maneuver.edit_preview().start_time_s = 240.0;

    track.cache = make_draw_ready_cache(state, 6u, 100.0, 500.0);
    set_planned_path(track.cache, 300.0, 380.0, 500.0);
    track.cache.identity.maneuver_plan_signature_valid = true;
    track.cache.identity.maneuver_plan_signature = old_plan_signature;

    track.authoritative_cache = make_draw_ready_cache(state, 5u, 100.0, 500.0);
    set_planned_path(track.authoritative_cache, 100.0, 240.0, 500.0);
    track.authoritative_cache.identity.maneuver_plan_signature_valid = true;
    track.authoritative_cache.identity.maneuver_plan_signature = old_plan_signature;

    Game::PredictionDrawDetail::PredictionTrackDrawContext draw_ctx{};
    ASSERT_TRUE(make_prediction_adapter(state).build_orbit_prediction_track_draw_context(
            track,
            make_draw_global_context(100.0),
            draw_ctx));

    EXPECT_EQ(draw_ctx.stale_planned_cache, &track.authoritative_cache);
    EXPECT_TRUE(draw_ctx.stale_planned_cache_drawable);
    EXPECT_DOUBLE_EQ(draw_ctx.stale_planned_cache_prefix_cutoff_s, 240.0);

    GameplayTestHooks::clear_entities();
}

TEST(GameplayPredictionManeuverTests, DrawContextKeepsAuthoritativePlannedPrefixAfterEditRelease)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    Game::Entity player_entity{Game::EntityId{1}, "player"};
    register_player_draw_subject(state, player_entity);

    Game::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    node.dv_rtn_mps = glm::dvec3(0.0, 5.0, 0.0);
    state._maneuver.plan().selected_node_id = node.id;
    state._maneuver.plan().nodes.push_back(node);
    const uint64_t old_plan_signature = make_prediction_adapter(state).current_maneuver_plan_signature();
    state._maneuver.plan().nodes.front().dv_rtn_mps = glm::dvec3(0.0, 12.0, 0.0);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = node.id;
    track.preview_anchor.anchor_time_s = node.time_s;
    track.preview_anchor.visual_window_s = 120.0;
    track.preview_anchor.exact_window_s = 60.0;
    track.dirty = true;

    track.cache = make_draw_ready_cache(state, 5u, 100.0, 500.0);

    track.authoritative_cache = make_draw_ready_cache(state, 4u, 100.0, 500.0);
    set_planned_path(track.authoritative_cache, 100.0, node.time_s, 500.0);
    track.authoritative_cache.identity.maneuver_plan_signature_valid = true;
    track.authoritative_cache.identity.maneuver_plan_signature = old_plan_signature;

    Game::PredictionDrawDetail::PredictionTrackDrawContext draw_ctx{};
    ASSERT_TRUE(make_prediction_adapter(state).build_orbit_prediction_track_draw_context(
            track,
            make_draw_global_context(100.0),
            draw_ctx));

    EXPECT_FALSE(draw_ctx.planned_cache_current);
    EXPECT_FALSE(draw_ctx.planned_cache_drawable);
    EXPECT_EQ(draw_ctx.stale_planned_cache, &track.authoritative_cache);
    EXPECT_TRUE(draw_ctx.stale_planned_cache_drawable);
    EXPECT_DOUBLE_EQ(draw_ctx.stale_planned_cache_prefix_cutoff_s, node.time_s);
    EXPECT_EQ(draw_ctx.planned_window_assembly,
              &track.authoritative_cache.display.planned_chunk_assembly);
    EXPECT_TRUE(draw_ctx.planned_window_policy.valid);
    EXPECT_DOUBLE_EQ(draw_ctx.planned_window_policy.visual_window_start_time_s, 100.0);
    EXPECT_DOUBLE_EQ(draw_ctx.planned_window_policy.pick_window_start_time_s, 100.0);

    GameplayTestHooks::clear_entities();
}

TEST(GameplayPredictionManeuverTests, DrawTrackUsesFullStreamOverlayForActiveManeuverRefine)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    Game::Entity player_entity{Game::EntityId{1}, "player"};
    register_player_draw_subject(state, player_entity);

    Game::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    node.dv_rtn_mps = glm::dvec3(0.0, 5.0, 0.0);
    state._maneuver.plan().selected_node_id = node.id;
    state._maneuver.plan().nodes.push_back(node);
    const uint64_t old_plan_signature = make_prediction_adapter(state).current_maneuver_plan_signature();
    state._maneuver.plan().nodes.front().dv_rtn_mps = glm::dvec3(0.0, 12.0, 0.0);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = node.id;
    track.preview_anchor.anchor_time_s = node.time_s;
    track.preview_anchor.visual_window_s = 120.0;
    track.preview_anchor.exact_window_s = 60.0;
    track.dirty = true;

    track.cache = make_draw_ready_cache(state, 5u, 100.0, 500.0);
    track.authoritative_cache = make_draw_ready_cache(state, 4u, 100.0, 500.0);
    set_planned_path(track.authoritative_cache, 100.0, node.time_s, 500.0);
    track.authoritative_cache.identity.maneuver_plan_signature_valid = true;
    track.authoritative_cache.identity.maneuver_plan_signature = old_plan_signature;

    track.full_stream_overlay.chunk_assembly.valid = true;
    track.full_stream_overlay.chunk_assembly.generation_id = track.cache.identity.generation_id;
    track.full_stream_overlay.display_frame_key = track.cache.display.display_frame_key;
    track.full_stream_overlay.display_frame_revision = track.cache.display.display_frame_revision;
    track.full_stream_overlay.chunk_assembly.chunks = {
            make_chunk(0u, track.cache.identity.generation_id, node.time_s, node.time_s + 60.0, 7'200'000.0, 7'280'000.0),
    };

    OrbitPlotSystem plot{};
    Game::PredictionDrawDetail::PredictionGlobalDrawContext global_ctx = make_draw_global_context(100.0);
    global_ctx.orbit_plot = &plot;

    Game::PredictionDrawDetail::PredictionTrackDrawContext draw_ctx{};
    ASSERT_TRUE(make_prediction_adapter(state).build_orbit_prediction_track_draw_context(track, global_ctx, draw_ctx));
    make_prediction_adapter(state).draw_orbit_prediction_track_windows(draw_ctx);

    EXPECT_EQ(state.prediction_for_test().orbit_plot_perf.planned_chunk_count, 1u);
    EXPECT_EQ(state.prediction_for_test().orbit_plot_perf.planned_chunks_drawn, 1u);

    GameplayTestHooks::clear_entities();
}

TEST(GameplayPredictionManeuverTests, DrawPlannerBuildsRenderAndPickWindowsBeforeEmission)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    Game::Entity player_entity{Game::EntityId{1}, "player"};
    register_player_draw_subject(state, player_entity);

    Game::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    node.dv_rtn_mps = glm::dvec3(0.0, 5.0, 0.0);
    state._maneuver.plan().selected_node_id = node.id;
    state._maneuver.plan().nodes.push_back(node);
    const uint64_t plan_signature = make_prediction_adapter(state).current_maneuver_plan_signature();

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;
    track.cache = make_draw_ready_cache(state, 5u, 100.0, 500.0);
    set_planned_path(track.cache, node.time_s, 320.0, 500.0);
    track.cache.identity.maneuver_plan_signature_valid = true;
    track.cache.identity.maneuver_plan_signature = plan_signature;

    Game::GameplayPredictionAdapter adapter = make_prediction_adapter(state);
    Game::PredictionDrawDetail::PredictionTrackVisualPlan plan{};
    ASSERT_TRUE(Game::PredictionDrawDetail::PredictionDrawPlanner(adapter)
                        .build_track(track, make_draw_global_context(100.0), plan));

    EXPECT_TRUE(plan.base_future_draw_window.valid);
    EXPECT_FALSE(plan.track.maneuver_drag_active);
    EXPECT_TRUE(plan.track.use_planned_adaptive_curve);
    EXPECT_TRUE(plan.track.base_pick_window.valid);
    EXPECT_TRUE(plan.track.planned_draw_window.valid);
    EXPECT_TRUE(plan.track.planned_pick_window.valid);
    EXPECT_GT(plan.track.planned_pick_window.t1_s, plan.track.planned_pick_window.t0_s);

    GameplayTestHooks::clear_entities();
}

TEST(GameplayPredictionManeuverTests, DrawPlannerUsesPlannedAdaptiveCurveDuringPreviewFallback)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    Game::Entity player_entity{Game::EntityId{1}, "player"};
    register_player_draw_subject(state, player_entity);

    Game::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    node.dv_rtn_mps = glm::dvec3(0.0, 5.0, 0.0);
    state._maneuver.plan().selected_node_id = node.id;
    state._maneuver.plan().nodes.push_back(node);
    const uint64_t plan_signature = make_prediction_adapter(state).current_maneuver_plan_signature();

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::PreviewStreaming;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = node.id;
    track.preview_anchor.anchor_time_s = node.time_s;
    track.preview_anchor.visual_window_s = 120.0;
    track.preview_anchor.exact_window_s = 60.0;
    track.cache = make_draw_ready_cache(state, 5u, 100.0, 500.0);
    set_planned_path(track.cache, node.time_s, 320.0, 500.0);
    track.cache.identity.maneuver_plan_signature_valid = true;
    track.cache.identity.maneuver_plan_signature = plan_signature;

    Game::GameplayPredictionAdapter adapter = make_prediction_adapter(state);
    Game::PredictionDrawDetail::PredictionTrackVisualPlan plan{};
    ASSERT_TRUE(Game::PredictionDrawDetail::PredictionDrawPlanner(adapter)
                        .build_track(track, make_draw_global_context(100.0), plan));

    EXPECT_TRUE(plan.overlay_layers.preview_fallback_active);
    EXPECT_TRUE(plan.track.planned_cache_drawable);
    EXPECT_TRUE(plan.track.use_planned_adaptive_curve);
    EXPECT_TRUE(plan.track.planned_draw_window.valid);

    GameplayTestHooks::clear_entities();
}

TEST(GameplayPredictionManeuverTests, PickAnchorTimesOnlyIncludeNodesInsidePickWindows)
{
    std::vector<Game::ManeuverNode> nodes;
    for (const double t_s : {80.0, 150.0, 220.0, 320.0, 350.0, 420.0})
    {
        Game::ManeuverNode node{};
        node.time_s = t_s;
        nodes.push_back(node);
    }

    Game::PredictionDrawDetail::PickWindow base_window{};
    base_window.valid = true;
    base_window.t0_s = 100.0;
    base_window.t1_s = 200.0;

    Game::PredictionDrawDetail::PickWindow planned_window{};
    planned_window.valid = true;
    planned_window.t0_s = 300.0;
    planned_window.t1_s = 400.0;
    planned_window.anchor_time_s = 350.0;

    const std::vector<double> anchor_times =
            Game::PredictionDrawDetail::collect_pick_anchor_times(nodes, base_window, planned_window, 120.0);

    const std::vector<double> expected{120.0, 150.0, 320.0, 350.0};
    EXPECT_EQ(anchor_times, expected);
}

TEST(GameplayPredictionManeuverTests, PlannedCurveAnchorTimesUseCachedWindowedNodeTimes)
{
    std::vector<Game::ManeuverNode> nodes;
    for (const double t_s : {80.0, 150.0, 220.0, 320.0, 420.0})
    {
        Game::ManeuverNode node{};
        node.time_s = t_s;
        nodes.push_back(node);
    }

    Game::OrbitPredictionCache cache{};
    cache.identity.generation_id = 5u;
    cache.identity.maneuver_plan_revision = 4u;
    cache.solver.planned.maneuver_previews = {
            Game::OrbitPredictionService::ManeuverNodePreview{.node_id = 1, .t_s = 110.0, .valid = true},
            Game::OrbitPredictionService::ManeuverNodePreview{.node_id = 2, .t_s = 250.0, .valid = true},
            Game::OrbitPredictionService::ManeuverNodePreview{.node_id = 3, .t_s = 320.0, .valid = true},
            Game::OrbitPredictionService::ManeuverNodePreview{.node_id = 4, .t_s = 360.0, .valid = false},
            Game::OrbitPredictionService::ManeuverNodePreview{.node_id = 5, .t_s = 420.0, .valid = true},
    };

    Game::PredictionTimeAnchorCache maneuver_cache{};
    Game::PredictionTimeAnchorCache preview_cache{};

    const std::vector<double> wide_anchors =
            Game::PredictionDrawDetail::collect_planned_curve_anchor_times(maneuver_cache,
                                                                           preview_cache,
                                                                           nodes,
                                                                           9u,
                                                                           cache,
                                                                           350.0,
                                                                           true,
                                                                           100.0,
                                                                           400.0);
    const std::vector<double> expected_wide{100.0, 110.0, 150.0, 220.0, 250.0, 320.0, 350.0, 400.0};
    EXPECT_EQ(wide_anchors, expected_wide);
    EXPECT_EQ(maneuver_cache.revision, 9u);
    EXPECT_EQ(maneuver_cache.source_count, nodes.size());
    EXPECT_EQ(preview_cache.generation_id, 5u);
    EXPECT_EQ(preview_cache.revision, 4u);
    EXPECT_EQ(preview_cache.source_count, cache.solver.planned.maneuver_previews.size());

    const std::vector<double> narrow_anchors =
            Game::PredictionDrawDetail::collect_planned_curve_anchor_times(maneuver_cache,
                                                                           preview_cache,
                                                                           nodes,
                                                                           9u,
                                                                           cache,
                                                                           350.0,
                                                                           true,
                                                                           300.0,
                                                                           360.0);
    const std::vector<double> expected_narrow{300.0, 320.0, 350.0, 360.0};
    EXPECT_EQ(narrow_anchors, expected_narrow);
}

TEST(GameplayPredictionManeuverTests, DrawPlannerSnapshotsFullStreamOverlayForRenderAndPick)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    Game::Entity player_entity{Game::EntityId{1}, "player"};
    register_player_draw_subject(state, player_entity);

    Game::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    node.dv_rtn_mps = glm::dvec3(0.0, 5.0, 0.0);
    state._maneuver.plan().selected_node_id = node.id;
    state._maneuver.plan().nodes.push_back(node);
    const uint64_t old_plan_signature = make_prediction_adapter(state).current_maneuver_plan_signature();
    state._maneuver.plan().nodes.front().dv_rtn_mps = glm::dvec3(0.0, 12.0, 0.0);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = node.id;
    track.preview_anchor.anchor_time_s = node.time_s;
    track.preview_anchor.visual_window_s = 120.0;
    track.preview_anchor.exact_window_s = 60.0;
    track.dirty = true;
    track.cache = make_draw_ready_cache(state, 5u, 100.0, 500.0);
    track.authoritative_cache = make_draw_ready_cache(state, 4u, 100.0, 500.0);
    set_planned_path(track.authoritative_cache, 100.0, node.time_s, 500.0);
    track.authoritative_cache.identity.maneuver_plan_signature_valid = true;
    track.authoritative_cache.identity.maneuver_plan_signature = old_plan_signature;
    track.full_stream_overlay.chunk_assembly.valid = true;
    track.full_stream_overlay.chunk_assembly.generation_id = track.cache.identity.generation_id;
    track.full_stream_overlay.display_frame_key = track.cache.display.display_frame_key;
    track.full_stream_overlay.display_frame_revision = track.cache.display.display_frame_revision;
    track.full_stream_overlay.chunk_assembly.chunks = {
            make_chunk(0u, track.cache.identity.generation_id, node.time_s, node.time_s + 60.0, 7'200'000.0, 7'280'000.0),
    };

    OrbitPlotSystem plot{};
    Game::PredictionDrawDetail::PredictionGlobalDrawContext global_ctx = make_draw_global_context(100.0);
    global_ctx.orbit_plot = &plot;

    Game::GameplayPredictionAdapter adapter = make_prediction_adapter(state);
    Game::PredictionDrawDetail::PredictionTrackVisualPlan plan{};
    ASSERT_TRUE(Game::PredictionDrawDetail::PredictionDrawPlanner(adapter).build_track(track, global_ctx, plan));
    EXPECT_FALSE(plan.track.maneuver_drag_active);
    ASSERT_TRUE(plan.full_stream_overlay_active);
    ASSERT_EQ(plan.full_stream_assembly.chunks.size(), 1u);

    Game::PredictionDrawDetail::PredictionRenderEmitter(adapter).emit(plan);

    EXPECT_EQ(state.prediction_for_test().orbit_plot_perf.planned_chunk_count, 1u);
    EXPECT_EQ(state.prediction_for_test().orbit_plot_perf.planned_chunks_drawn, 1u);

    GameplayTestHooks::clear_entities();
}

TEST(GameplayPredictionManeuverTests, DrawPlannerHidesFullStreamOverlayOnlyDuringActualManeuverDrag)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    Game::Entity player_entity{Game::EntityId{1}, "player"};
    register_player_draw_subject(state, player_entity);

    Game::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    node.dv_rtn_mps = glm::dvec3(0.0, 5.0, 0.0);
    state._maneuver.plan().selected_node_id = node.id;
    state._maneuver.plan().nodes.push_back(node);
    const uint64_t old_plan_signature = make_prediction_adapter(state).current_maneuver_plan_signature();
    state._maneuver.plan().nodes.front().dv_rtn_mps = glm::dvec3(0.0, 12.0, 0.0);
    state._maneuver.gizmo_interaction().state = Game::ManeuverGizmoInteraction::State::DragAxis;
    state._maneuver.gizmo_interaction().node_id = node.id;

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = node.id;
    track.preview_anchor.anchor_time_s = node.time_s;
    track.preview_anchor.visual_window_s = 120.0;
    track.preview_anchor.exact_window_s = 60.0;
    track.dirty = true;
    track.cache = make_draw_ready_cache(state, 5u, 100.0, 500.0);
    track.authoritative_cache = make_draw_ready_cache(state, 4u, 100.0, 500.0);
    set_planned_path(track.authoritative_cache, 100.0, node.time_s, 500.0);
    track.authoritative_cache.identity.maneuver_plan_signature_valid = true;
    track.authoritative_cache.identity.maneuver_plan_signature = old_plan_signature;
    track.full_stream_overlay.chunk_assembly.valid = true;
    track.full_stream_overlay.chunk_assembly.generation_id = track.cache.identity.generation_id;
    track.full_stream_overlay.display_frame_key = track.cache.display.display_frame_key;
    track.full_stream_overlay.display_frame_revision = track.cache.display.display_frame_revision;
    track.full_stream_overlay.chunk_assembly.chunks = {
            make_chunk(0u, track.cache.identity.generation_id, node.time_s, node.time_s + 60.0, 7'200'000.0, 7'280'000.0),
    };

    Game::GameplayPredictionAdapter adapter = make_prediction_adapter(state);
    Game::PredictionDrawDetail::PredictionTrackVisualPlan plan{};
    ASSERT_TRUE(Game::PredictionDrawDetail::PredictionDrawPlanner(adapter).build_track(
            track,
            make_draw_global_context(100.0),
            plan));

    EXPECT_TRUE(plan.track.maneuver_drag_active);
    EXPECT_FALSE(plan.full_stream_overlay_active);

    GameplayTestHooks::clear_entities();
}

TEST(GameplayPredictionManeuverTests, DrawPlannerShowsFullStreamOverlayDuringPreviewFallbackWhenNotDragging)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    Game::Entity player_entity{Game::EntityId{1}, "player"};
    register_player_draw_subject(state, player_entity);

    Game::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    node.dv_rtn_mps = glm::dvec3(0.0, 5.0, 0.0);
    state._maneuver.plan().selected_node_id = node.id;
    state._maneuver.plan().nodes.push_back(node);
    const uint64_t old_plan_signature = make_prediction_adapter(state).current_maneuver_plan_signature();
    state._maneuver.plan().nodes.front().dv_rtn_mps = glm::dvec3(0.0, 12.0, 0.0);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::PreviewStreaming;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = node.id;
    track.preview_anchor.anchor_time_s = node.time_s;
    track.preview_anchor.visual_window_s = 120.0;
    track.preview_anchor.exact_window_s = 60.0;
    track.dirty = true;
    track.cache = make_draw_ready_cache(state, 5u, 100.0, 500.0);
    track.authoritative_cache = make_draw_ready_cache(state, 4u, 100.0, 500.0);
    set_planned_path(track.authoritative_cache, 100.0, node.time_s, 500.0);
    track.authoritative_cache.identity.maneuver_plan_signature_valid = true;
    track.authoritative_cache.identity.maneuver_plan_signature = old_plan_signature;
    track.full_stream_overlay.chunk_assembly.valid = true;
    track.full_stream_overlay.chunk_assembly.generation_id = track.cache.identity.generation_id;
    track.full_stream_overlay.display_frame_key = track.cache.display.display_frame_key;
    track.full_stream_overlay.display_frame_revision = track.cache.display.display_frame_revision;
    track.full_stream_overlay.chunk_assembly.chunks = {
            make_chunk(0u, track.cache.identity.generation_id, node.time_s, node.time_s + 60.0, 7'200'000.0, 7'280'000.0),
    };

    Game::GameplayPredictionAdapter adapter = make_prediction_adapter(state);
    Game::PredictionDrawDetail::PredictionTrackVisualPlan plan{};
    ASSERT_TRUE(Game::PredictionDrawDetail::PredictionDrawPlanner(adapter).build_track(
            track,
            make_draw_global_context(100.0),
            plan));

    EXPECT_TRUE(plan.overlay_layers.preview_fallback_active);
    EXPECT_FALSE(plan.track.maneuver_drag_active);
    EXPECT_TRUE(plan.full_stream_overlay_active);
    EXPECT_EQ(plan.full_stream_assembly.chunks.size(), 1u);

    GameplayTestHooks::clear_entities();
}

TEST(GameplayPredictionManeuverTests, PickCacheInvalidatesOnIdentityWindowBudgetAndAdaptiveChanges)
{
    Game::PredictionLinePickCache cache{};
    const uint64_t generation_id = 5u;
    const uint64_t frame_key = 11u;
    const uint64_t frame_revision = 3u;
    const WorldVec3 ref_body_world{1.0, 2.0, 3.0};
    const WorldVec3 align_delta{4.0, 5.0, 6.0};
    const glm::dmat3 frame_to_world{1.0};
    const glm::dvec3 camera_world{7.0, 8.0, 9.0};
    Game::OrbitRenderCurve::FrustumContext frustum{};
    frustum.valid = true;
    frustum.viewproj = glm::mat4(1.0f);
    frustum.origin_world = WorldVec3(camera_world);

    Game::PredictionDrawDetail::mark_pick_cache_valid(cache,
                                                      generation_id,
                                                      frame_key,
                                                      frame_revision,
                                                      ref_body_world,
                                                      frame_to_world,
                                                      align_delta,
                                                      camera_world,
                                                      1.0,
                                                      720.0,
                                                      0.75,
                                                      frustum,
                                                      0.05,
                                                      100.0,
                                                      200.0,
                                                      128u,
                                                      true,
                                                      true);

    EXPECT_FALSE(Game::PredictionDrawDetail::should_rebuild_pick_cache(cache,
                                                                       generation_id,
                                                                       frame_key,
                                                                       frame_revision,
                                                                       ref_body_world,
                                                                       frame_to_world,
                                                                       align_delta,
                                                                       camera_world,
                                                                       1.0,
                                                                       720.0,
                                                                       0.75,
                                                                       frustum,
                                                                       0.05,
                                                                       100.0,
                                                                       200.0,
                                                                       128u,
                                                                       true,
                                                                       true));
    EXPECT_TRUE(Game::PredictionDrawDetail::should_rebuild_pick_cache(cache,
                                                                      generation_id + 1u,
                                                                      frame_key,
                                                                      frame_revision,
                                                                      ref_body_world,
                                                                      frame_to_world,
                                                                      align_delta,
                                                                      camera_world,
                                                                      1.0,
                                                                      720.0,
                                                                      0.75,
                                                                      frustum,
                                                                      0.05,
                                                                      100.0,
                                                                      200.0,
                                                                      128u,
                                                                      true,
                                                                      true));
    EXPECT_TRUE(Game::PredictionDrawDetail::should_rebuild_pick_cache(cache,
                                                                      generation_id,
                                                                      frame_key,
                                                                      frame_revision + 1u,
                                                                      ref_body_world,
                                                                      frame_to_world,
                                                                      align_delta,
                                                                      camera_world,
                                                                      1.0,
                                                                      720.0,
                                                                      0.75,
                                                                      frustum,
                                                                      0.05,
                                                                      100.0,
                                                                      200.0,
                                                                      128u,
                                                                      true,
                                                                      true));
    EXPECT_TRUE(Game::PredictionDrawDetail::should_rebuild_pick_cache(cache,
                                                                      generation_id,
                                                                      frame_key,
                                                                      frame_revision,
                                                                      ref_body_world,
                                                                      frame_to_world,
                                                                      align_delta,
                                                                      camera_world,
                                                                      1.0,
                                                                      720.0,
                                                                      0.75,
                                                                      frustum,
                                                                      0.05,
                                                                      100.0,
                                                                      220.0,
                                                                      128u,
                                                                      true,
                                                                      true));
    EXPECT_TRUE(Game::PredictionDrawDetail::should_rebuild_pick_cache(cache,
                                                                      generation_id,
                                                                      frame_key,
                                                                      frame_revision,
                                                                      ref_body_world,
                                                                      frame_to_world,
                                                                      align_delta,
                                                                      camera_world,
                                                                      1.0,
                                                                      720.0,
                                                                      0.75,
                                                                      frustum,
                                                                      0.05,
                                                                      100.0,
                                                                      200.0,
                                                                      256u,
                                                                      true,
                                                                      true));
    EXPECT_TRUE(Game::PredictionDrawDetail::should_rebuild_pick_cache(cache,
                                                                      generation_id,
                                                                      frame_key,
                                                                      frame_revision,
                                                                      ref_body_world,
                                                                      frame_to_world,
                                                                      align_delta,
                                                                      camera_world,
                                                                      1.0,
                                                                      720.0,
                                                                      0.75,
                                                                      frustum,
                                                                      0.05,
                                                                      100.0,
                                                                      200.0,
                                                                      128u,
                                                                      false,
                                                                      true));
}

TEST(GameplayPredictionManeuverTests, DrawTrackUsesStalePrefixWhenAwaitingFullRefineHasNoOverlay)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    state.prediction_for_test().draw_full_orbit = false;
    state.prediction_for_test().draw_future_segment = false;
    state.prediction_for_test().draw_config.draw_planned_as_dashed = false;
    Game::Entity player_entity{Game::EntityId{1}, "player"};
    register_player_draw_subject(state, player_entity);

    Game::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    node.dv_rtn_mps = glm::dvec3(0.0, 5.0, 0.0);
    state._maneuver.plan().selected_node_id = node.id;
    state._maneuver.plan().nodes.push_back(node);
    const uint64_t old_plan_signature = make_prediction_adapter(state).current_maneuver_plan_signature();
    state._maneuver.plan().nodes.front().dv_rtn_mps = glm::dvec3(0.0, 12.0, 0.0);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = node.id;
    track.preview_anchor.anchor_time_s = node.time_s;
    track.preview_anchor.visual_window_s = 120.0;
    track.preview_anchor.exact_window_s = 60.0;
    track.dirty = true;

    track.cache = make_draw_ready_cache(state, 5u, 100.0, 500.0);
    track.authoritative_cache = make_draw_ready_cache(state, 4u, 100.0, 500.0);
    set_planned_path(track.authoritative_cache, 100.0, node.time_s, 500.0);
    track.authoritative_cache.identity.maneuver_plan_signature_valid = true;
    track.authoritative_cache.identity.maneuver_plan_signature = old_plan_signature;

    OrbitPlotSystem plot{};
    Game::PredictionDrawDetail::PredictionGlobalDrawContext global_ctx = make_draw_global_context(100.0);
    global_ctx.orbit_plot = &plot;

    Game::PredictionDrawDetail::PredictionTrackDrawContext draw_ctx{};
    ASSERT_TRUE(make_prediction_adapter(state).build_orbit_prediction_track_draw_context(track, global_ctx, draw_ctx));
    ASSERT_EQ(draw_ctx.stale_planned_cache, &track.authoritative_cache);
    ASSERT_TRUE(draw_ctx.stale_planned_cache_drawable);

    draw_ctx.direct_world_polyline = true;
    make_prediction_adapter(state).draw_orbit_prediction_track_windows(draw_ctx);

    EXPECT_GT(plot.stats().pending_line_count, 0u);

    GameplayTestHooks::clear_entities();
}

TEST(GameplayPredictionManeuverTests, RuntimeCacheKeepsCachedNodeGizmoAfterEditRelease)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());
    Game::Entity player_entity{Game::EntityId{1}, "player"};
    register_player_draw_subject(state, player_entity);

    Game::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    node.dv_rtn_mps = glm::dvec3(0.0, 12.0, 0.0);
    node.position_world = WorldVec3(7'240'000.0, 11.0, 0.0);
    node.basis_r_world = glm::dvec3(1.0, 0.0, 0.0);
    node.basis_t_world = glm::dvec3(0.0, 1.0, 0.0);
    node.basis_n_world = glm::dvec3(0.0, 0.0, 1.0);
    node.maneuver_basis_r_world = node.basis_r_world;
    node.maneuver_basis_t_world = node.basis_t_world;
    node.maneuver_basis_n_world = node.basis_n_world;
    node.gizmo_valid = true;
    state._maneuver.plan().selected_node_id = node.id;
    state._maneuver.plan().nodes.push_back(node);

    Game::ManeuverNode downstream{};
    downstream.id = 8;
    downstream.time_s = 300.0;
    downstream.dv_rtn_mps = glm::dvec3(0.0, 1.0, 0.0);
    state._maneuver.plan().nodes.push_back(downstream);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = node.id;
    track.preview_anchor.anchor_time_s = node.time_s;
    track.dirty = true;
    track.cache = make_draw_ready_cache(state, 5u, 100.0, 500.0);
    state.prediction_for_test().tracks.push_back(track);

    Game::GameStateContext ctx{};
    state.refresh_maneuver_node_runtime_cache(ctx);

    ASSERT_EQ(state._maneuver.plan().nodes.size(), 2u);
    EXPECT_TRUE(state._maneuver.plan().nodes.front().gizmo_valid);
    EXPECT_DOUBLE_EQ(state._maneuver.plan().nodes.front().position_world.x, node.position_world.x);
    EXPECT_DOUBLE_EQ(state._maneuver.plan().nodes.front().position_world.y, node.position_world.y);

    GameplayTestHooks::clear_entities();
}

TEST(GameplayPredictionManeuverTests, RuntimeCacheShowsSelectedNewNodeBeforePlannedTrajectoryIsReady)
{
    Game::GameplayState state{};
    Game::OrbitPredictionCache cache = make_draw_ready_cache(state, 5u, 100.0, 500.0);

    orbitsim::MassiveBody primary{};
    primary.id = 1;
    primary.mass_kg = 5.972e24;
    primary.radius_m = 6'371'000.0;
    primary.state = orbitsim::make_state(glm::dvec3(0.0), glm::dvec3(0.0));
    cache.solver.core.massive_bodies.push_back(primary);

    Game::ManeuverPlanState plan{};
    Game::ManeuverNode first{};
    first.id = 7;
    first.time_s = 180.0;
    first.primary_body_auto = false;
    first.primary_body_id = primary.id;
    plan.nodes.push_back(first);

    Game::ManeuverNode selected{};
    selected.id = 8;
    selected.time_s = 240.0;
    selected.primary_body_auto = false;
    selected.primary_body_id = primary.id;
    plan.selected_node_id = selected.id;
    plan.nodes.push_back(selected);

    Game::ManeuverGizmoInteraction interaction{};
    Game::ManeuverNodeEditPreview edit_preview{};
    const Game::ManeuverRuntimeCacheInput input{
            .plan = plan,
            .active_cache = &cache,
            .gizmo_interaction = interaction,
            .edit_preview = edit_preview,
            .basis_mode = Game::ManeuverGizmoBasisMode::RTN,
            .display_time_s = 100.0,
            .current_sim_time_s = 100.0,
            .resolve_primary_body_id = [primary](const Game::ManeuverNode &, double) {
                return primary.id;
            },
    };

    const Game::ManeuverRuntimeCacheResult result = Game::ManeuverRuntimeCacheBuilder::rebuild(input);

    EXPECT_TRUE(result.active_prediction_cache_valid);
    EXPECT_EQ(result.valid_node_count, 1u);
    const Game::ManeuverNode *refreshed = plan.find_node(selected.id);
    ASSERT_NE(refreshed, nullptr);
    EXPECT_TRUE(refreshed->gizmo_valid);
    const glm::dvec3 expected_position = Game::OrbitPredictionMath::sample_pair_position_m(
            cache.display.trajectory_frame.front(),
            cache.display.trajectory_frame.back(),
            selected.time_s);
    EXPECT_NEAR(refreshed->position_world.x, expected_position.x, 1.0);
    EXPECT_NEAR(refreshed->position_world.y, expected_position.y, 1.0);
}

TEST(GameplayPredictionManeuverTests, RuntimeCacheRebuildsDisplayBasisDuringCachedReleaseHold)
{
    Game::GameplayState state{};
    Game::OrbitPredictionCache cache = make_draw_ready_cache(state, 5u, 100.0, 500.0);

    Game::ManeuverPlanState plan{};
    Game::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    node.dv_rtn_mps = glm::dvec3(0.0, 12.0, 0.0);
    node.position_world = WorldVec3(7'240'000.0, 11.0, 0.0);
    node.maneuver_basis_r_world = glm::dvec3(1.0, 0.0, 0.0);
    node.maneuver_basis_t_world = glm::dvec3(0.0, 1.0, 0.0);
    node.maneuver_basis_n_world = glm::dvec3(0.0, 0.0, 1.0);
    node.basis_r_world = glm::dvec3(0.0, 1.0, 0.0);
    node.basis_t_world = glm::dvec3(1.0, 0.0, 0.0);
    node.basis_n_world = glm::dvec3(0.0, 0.0, 1.0);
    node.gizmo_valid = true;
    plan.selected_node_id = node.id;
    plan.nodes.push_back(node);

    Game::PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot lifecycle{};
    lifecycle.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    Game::ManeuverGizmoInteraction interaction{};
    Game::ManeuverNodeEditPreview edit_preview{};
    const Game::ManeuverRuntimeCacheInput input{
            .plan = plan,
            .active_cache = &cache,
            .lifecycle = lifecycle,
            .gizmo_interaction = interaction,
            .edit_preview = edit_preview,
            .basis_mode = Game::ManeuverGizmoBasisMode::RTN,
            .display_time_s = 100.0,
            .current_sim_time_s = 100.0,
            .hold_cached_release_state = true,
    };

    const Game::ManeuverRuntimeCacheResult result = Game::ManeuverRuntimeCacheBuilder::rebuild(input);

    ASSERT_EQ(result.valid_node_count, 1u);
    ASSERT_EQ(plan.nodes.size(), 1u);
    EXPECT_TRUE(plan.nodes.front().gizmo_valid);
    EXPECT_DOUBLE_EQ(plan.nodes.front().basis_r_world.x, plan.nodes.front().maneuver_basis_r_world.x);
    EXPECT_DOUBLE_EQ(plan.nodes.front().basis_r_world.y, plan.nodes.front().maneuver_basis_r_world.y);
    EXPECT_DOUBLE_EQ(plan.nodes.front().basis_r_world.z, plan.nodes.front().maneuver_basis_r_world.z);
    EXPECT_DOUBLE_EQ(plan.nodes.front().basis_t_world.x, plan.nodes.front().maneuver_basis_t_world.x);
    EXPECT_DOUBLE_EQ(plan.nodes.front().basis_t_world.y, plan.nodes.front().maneuver_basis_t_world.y);
    EXPECT_DOUBLE_EQ(plan.nodes.front().basis_t_world.z, plan.nodes.front().maneuver_basis_t_world.z);
    EXPECT_DOUBLE_EQ(plan.nodes.front().basis_n_world.x, plan.nodes.front().maneuver_basis_n_world.x);
    EXPECT_DOUBLE_EQ(plan.nodes.front().basis_n_world.y, plan.nodes.front().maneuver_basis_n_world.y);
    EXPECT_DOUBLE_EQ(plan.nodes.front().basis_n_world.z, plan.nodes.front().maneuver_basis_n_world.z);
}

TEST(GameplayPredictionManeuverTests, RuntimeCacheForceRefreshBypassesCachedReleaseHoldForPonBasis)
{
    Game::GameplayState state{};
    Game::OrbitPredictionCache cache = make_draw_ready_cache(state, 5u, 100.0, 500.0);
    for (orbitsim::TrajectorySample &sample : cache.display.trajectory_frame)
    {
        sample.velocity_mps = orbitsim::Vec3{1'000.0, 0.0, 0.0};
    }

    orbitsim::MassiveBody primary{};
    primary.id = 1;
    primary.mass_kg = 5.972e24;
    primary.radius_m = 6'371'000.0;
    primary.state = orbitsim::make_state(glm::dvec3(0.0), glm::dvec3(0.0));
    cache.solver.core.massive_bodies.push_back(primary);

    Game::ManeuverPlanState plan{};
    Game::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    node.dv_rtn_mps = glm::dvec3(0.0, 12.0, 0.0);
    node.position_world = WorldVec3(7'240'000.0, 11.0, 0.0);
    node.maneuver_basis_r_world = glm::dvec3(1.0, 0.0, 0.0);
    node.maneuver_basis_t_world = glm::dvec3(0.0, 1.0, 0.0);
    node.maneuver_basis_n_world = glm::dvec3(0.0, 0.0, 1.0);
    node.basis_r_world = node.maneuver_basis_r_world;
    node.basis_t_world = node.maneuver_basis_t_world;
    node.basis_n_world = node.maneuver_basis_n_world;
    node.gizmo_valid = true;
    plan.selected_node_id = node.id;
    plan.nodes.push_back(node);

    Game::PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot lifecycle{};
    lifecycle.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    Game::ManeuverGizmoInteraction interaction{};
    Game::ManeuverNodeEditPreview edit_preview{};
    const Game::ManeuverRuntimeCacheInput input{
            .plan = plan,
            .active_cache = &cache,
            .lifecycle = lifecycle,
            .gizmo_interaction = interaction,
            .edit_preview = edit_preview,
            .basis_mode = Game::ManeuverGizmoBasisMode::ProgradeOutwardNormal,
            .display_time_s = 100.0,
            .current_sim_time_s = 100.0,
            .hold_cached_release_state = true,
            .force_display_basis_refresh = true,
            .resolve_primary_body_id = [primary](const Game::ManeuverNode &, double) {
                return primary.id;
            },
    };

    const Game::ManeuverRuntimeCacheResult result = Game::ManeuverRuntimeCacheBuilder::rebuild(input);

    ASSERT_EQ(result.valid_node_count, 1u);
    ASSERT_EQ(plan.nodes.size(), 1u);
    EXPECT_TRUE(plan.nodes.front().gizmo_valid);
    EXPECT_NEAR(plan.nodes.front().basis_t_world.x, 1.0, 1.0e-9);
    EXPECT_NEAR(plan.nodes.front().basis_t_world.y, 0.0, 1.0e-9);
    EXPECT_NEAR(plan.nodes.front().basis_t_world.z, 0.0, 1.0e-9);
    EXPECT_NEAR(plan.nodes.front().basis_r_world.x, 0.0, 1.0e-9);
    EXPECT_NEAR(plan.nodes.front().basis_r_world.y, 1.0, 1.0e-9);
    EXPECT_NEAR(plan.nodes.front().basis_r_world.z, 0.0, 1.0e-9);
    EXPECT_NEAR(plan.nodes.front().basis_n_world.x, 0.0, 1.0e-9);
    EXPECT_NEAR(plan.nodes.front().basis_n_world.y, 0.0, 1.0e-9);
    EXPECT_NEAR(plan.nodes.front().basis_n_world.z, -1.0, 1.0e-9);

    const Game::ManeuverRuntimeCacheInput held_input{
            .plan = plan,
            .active_cache = &cache,
            .lifecycle = lifecycle,
            .gizmo_interaction = interaction,
            .edit_preview = edit_preview,
            .basis_mode = Game::ManeuverGizmoBasisMode::ProgradeOutwardNormal,
            .display_time_s = 100.0,
            .current_sim_time_s = 100.0,
            .hold_cached_release_state = true,
            .resolve_primary_body_id = [primary](const Game::ManeuverNode &, double) {
                return primary.id;
            },
    };

    const Game::ManeuverRuntimeCacheResult held_result = Game::ManeuverRuntimeCacheBuilder::rebuild(held_input);

    ASSERT_EQ(held_result.valid_node_count, 1u);
    ASSERT_EQ(plan.nodes.size(), 1u);
    EXPECT_TRUE(plan.nodes.front().gizmo_valid);
    EXPECT_NEAR(plan.nodes.front().basis_t_world.x, 1.0, 1.0e-9);
    EXPECT_NEAR(plan.nodes.front().basis_t_world.y, 0.0, 1.0e-9);
    EXPECT_NEAR(plan.nodes.front().basis_t_world.z, 0.0, 1.0e-9);
    EXPECT_NEAR(plan.nodes.front().basis_r_world.x, 0.0, 1.0e-9);
    EXPECT_NEAR(plan.nodes.front().basis_r_world.y, 1.0, 1.0e-9);
    EXPECT_NEAR(plan.nodes.front().basis_r_world.z, 0.0, 1.0e-9);
    EXPECT_NEAR(plan.nodes.front().basis_n_world.x, 0.0, 1.0e-9);
    EXPECT_NEAR(plan.nodes.front().basis_n_world.y, 0.0, 1.0e-9);
    EXPECT_NEAR(plan.nodes.front().basis_n_world.z, -1.0, 1.0e-9);
}

TEST(GameplayPredictionManeuverTests, FullRequestKeepsFullStreamPublishDisabledForStableActivePlayerWithManeuvers)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());

    Game::OrbiterInfo player{};
    player.entity = Game::EntityId{1};
    player.is_player = true;
    state._orbit.orbiters().push_back(player);
    state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};

    Game::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    state._maneuver.plan().nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    EXPECT_EQ(request.options.solve_quality, Game::OrbitPredictionService::SolveQuality::Full);
    EXPECT_EQ(request.maneuver.maneuver_impulses.size(), 1u);
    EXPECT_FALSE(request.maneuver.full_stream_publish.active);
}

TEST(GameplayPredictionManeuverTests, FullRequestEnablesFullStreamPublishForPostPreviewRefineWithPendingDerivedWork)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());

    Game::OrbiterInfo player{};
    player.entity = Game::EntityId{1};
    player.is_player = true;
    state._orbit.orbiters().push_back(player);
    state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};

    Game::ManeuverNode node{};
    node.id = 7;
    node.time_s = 240.0;
    state._maneuver.plan().nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = state.prediction_for_test().selection.active_subject;
    track.supports_maneuvers = true;
    track.cache = make_prediction_cache(4u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.authoritative_cache = track.cache;
    track.dirty = true;
    track.derived_request_pending = true;
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.preview_anchor.valid = true;
    track.preview_anchor.anchor_node_id = node.id;
    track.preview_anchor.anchor_time_s = node.time_s;

    Game::OrbitPredictionService::Request request{};
    const bool built = build_orbiter_prediction_request(state, track,
                                                              WorldVec3(7'000'000.0, 0.0, 0.0),
                                                              glm::dvec3(0.0, 7'500.0, 0.0),
                                                              100.0,
                                                              false,
                                                              true,
                                                              request);

    ASSERT_TRUE(built);
    EXPECT_EQ(request.options.solve_quality, Game::OrbitPredictionService::SolveQuality::Full);
    EXPECT_EQ(request.maneuver.maneuver_impulses.size(), 1u);
    EXPECT_FALSE(request.maneuver.planned_suffix_refine.active);
    EXPECT_TRUE(request.maneuver.full_stream_publish.active);
}

TEST(GameplayPredictionManeuverTests, FullRequestKeepsFullStreamPublishDisabledOutsideActivePlayerManeuverRefine)
{
    Game::GameplayState state{};
    state._orbit.scenario_owner() = make_reference_orbitsim(100.0);
    ASSERT_TRUE(state._orbit.scenario_owner());

    Game::OrbiterInfo player{};
    player.entity = Game::EntityId{1};
    player.is_player = true;
    state._orbit.orbiters().push_back(player);
    state.prediction_for_test().selection.active_subject = {Game::PredictionSubjectKind::Orbiter, 1};

    Game::ManeuverNode node{};
    node.id = 9;
    node.time_s = 240.0;
    state._maneuver.plan().nodes.push_back(node);

    Game::PredictionTrackState active_track{};
    active_track.key = state.prediction_for_test().selection.active_subject;
    active_track.supports_maneuvers = true;

    Game::OrbitPredictionService::Request no_maneuver_request{};
    ASSERT_TRUE(build_orbiter_prediction_request(state, active_track,
                                                       WorldVec3(7'000'000.0, 0.0, 0.0),
                                                       glm::dvec3(0.0, 7'500.0, 0.0),
                                                       100.0,
                                                       false,
                                                       false,
                                                       no_maneuver_request));
    EXPECT_FALSE(no_maneuver_request.maneuver.full_stream_publish.active);

    Game::PredictionTrackState overlay_track{};
    overlay_track.key = {Game::PredictionSubjectKind::Orbiter, 2};
    overlay_track.supports_maneuvers = true;

    Game::OrbitPredictionService::Request overlay_request{};
    ASSERT_TRUE(build_orbiter_prediction_request(state, overlay_track,
                                                       WorldVec3(7'000'000.0, 0.0, 0.0),
                                                       glm::dvec3(0.0, 7'500.0, 0.0),
                                                       100.0,
                                                       false,
                                                       true,
                                                       overlay_request));
    EXPECT_EQ(overlay_request.options.solve_quality, Game::OrbitPredictionService::SolveQuality::Full);
    EXPECT_EQ(overlay_request.maneuver.maneuver_impulses.size(), 1u);
    EXPECT_FALSE(overlay_request.maneuver.full_stream_publish.active);
}

TEST(GameplayPredictionManeuverTests, ShouldRebuildPredictionTrackWhenCoverageFallsShort)
{
    Game::GameplayState state{};
    state.prediction_for_test().draw_future_segment = true;
    state.prediction_for_test().sampling_policy.orbiter_min_window_s = 120.0;

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.cache.identity.valid = true;
    track.cache.identity.build_time_s = 0.0;
    track.cache.solver.base.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(60.0, 7'050'000.0)};
    track.cache.solver.base.trajectory_segments_inertial = {make_segment(0.0, 60.0, 7'000'000.0, 7'050'000.0)};

    EXPECT_TRUE(make_prediction_adapter(state).should_rebuild_prediction_track(track, 10.0, 0.016f, false, false));
}

TEST(GameplayPredictionManeuverTests, ThrustRefreshGatePreventsPerTickPredictionRebuilds)
{
    Game::GameplayState state{};
    state.prediction_for_test().draw_future_segment = true;
    state.prediction_for_test().sampling_policy.orbiter_min_window_s = 120.0;
    state.prediction_for_test().thrust_refresh_s = 0.1;

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.cache = make_draw_ready_cache(state, 5u, 100.0, 1'000.0);
    track.dirty = false;

    EXPECT_FALSE(make_prediction_adapter(state).should_rebuild_prediction_track(track, 100.05, 0.016f, true, false));
    EXPECT_TRUE(make_prediction_adapter(state).should_rebuild_prediction_track(track, 100.11, 0.016f, true, false));
}

TEST(GameplayPredictionManeuverTests, ShouldRebuildPredictionTrackWhenManeuverCoverageFallsShortOutsideLivePreview)
{
    Game::GameplayState state{};
    state.prediction_for_test().draw_future_segment = true;
    state.prediction_for_test().sampling_policy.orbiter_min_window_s = 120.0;
    state._maneuver.settings().plan_windows.solve_margin_s = 300.0;
    state._maneuver.settings().live_preview_active = false;

    Game::ManeuverNode node{};
    node.id = 1;
    node.time_s = 400.0;
    state._maneuver.plan().nodes.push_back(node);

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.cache.identity.valid = true;
    track.cache.identity.build_time_s = 0.0;
    track.cache.solver.base.trajectory_inertial = {make_sample(0.0, 7'000'000.0), make_sample(60.0, 7'050'000.0)};
    track.cache.solver.base.trajectory_segments_inertial = {make_segment(0.0, 60.0, 7'000'000.0, 7'050'000.0)};

    EXPECT_TRUE(make_prediction_adapter(state).should_rebuild_prediction_track(track, 10.0, 0.016f, false, true));
}

TEST(GameplayPredictionManeuverTests, AwaitFullRefineWaitsForPendingWork)
{
    Game::GameplayState state{};
    state.prediction_for_test().sampling_policy.orbiter_min_window_s = 5.0;

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.preview_state = Game::PredictionPreviewRuntimeState::AwaitFullRefine;
    track.cache = make_prediction_cache(5u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.dirty = false;
    track.derived_request_pending = true;

    EXPECT_FALSE(make_prediction_adapter(state).should_rebuild_prediction_track(track, 10.0, 0.016f, false, false));

    track.derived_request_pending = false;
    EXPECT_TRUE(make_prediction_adapter(state).should_rebuild_prediction_track(track, 10.0, 0.016f, false, false));
}

TEST(GameplayPredictionManeuverTests, PendingInvalidationAloneDoesNotForceImmediateRebuild)
{
    Game::GameplayState state{};
    state.prediction_for_test().sampling_policy.orbiter_min_window_s = 5.0;

    Game::PredictionTrackState track{};
    track.key = {Game::PredictionSubjectKind::Orbiter, 1};
    track.cache = make_prediction_cache(5u, 0.0, 20.0, 7'000'000.0, 7'200'000.0);
    track.dirty = false;
    track.request_pending = true;
    track.invalidated_while_pending = true;

    EXPECT_FALSE(make_prediction_adapter(state).should_rebuild_prediction_track(track, 10.0, 0.016f, false, false));
}
