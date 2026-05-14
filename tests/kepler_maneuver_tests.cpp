#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_system.h"
#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_orbit_pick.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <string_view>
#include <utility>

namespace
{
    constexpr orbitsim::BodyId kEarthId = 42;
    constexpr double kEarthMu = 3.986004418e14;
    constexpr double kOrbitRadiusM = 7'000'000.0;

    Game::KeplerManeuverEditorNode make_node(const int id,
                                             const double time_s,
                                             const orbitsim::Vec3 &dv_rtn_mps = {0.0, 0.0, 0.0})
    {
        Game::KeplerManeuverEditorNode node{};
        node.id = id;
        node.time_s = time_s;
        node.dv_rtn_mps = dv_rtn_mps;
        return node;
    }

    Game::KeplerOrbitArc make_circular_arc(const double t0_s,
                                           const double t1_s,
                                           const orbitsim::State &primary_state)
    {
        const double circular_speed_mps = std::sqrt(kEarthMu / kOrbitRadiusM);

        Game::KeplerOrbitArc arc{};
        arc.arc.mu_m3_s2 = kEarthMu;
        arc.arc.primary_body_id = kEarthId;
        arc.arc.t0_s = t0_s;
        arc.arc.t1_s = t1_s;
        arc.arc.state0_relative =
                orbitsim::make_state({kOrbitRadiusM, 0.0, 0.0},
                                      {0.0, circular_speed_mps, 0.0});
        arc.primary_state_inertial_at_t0 = primary_state;
        return arc;
    }

    Game::KeplerOrbitArc make_hyperbolic_arc(const double t0_s = 0.0,
                                             const double t1_s = 10'000.0)
    {
        Game::KeplerOrbitArc arc{};
        arc.arc.mu_m3_s2 = kEarthMu;
        arc.arc.primary_body_id = kEarthId;
        arc.arc.t0_s = t0_s;
        arc.arc.t1_s = t1_s;
        arc.arc.state0_relative =
                orbitsim::make_state({kOrbitRadiusM, 0.0, 0.0},
                                      {0.0, 12'000.0, 0.0});
        arc.primary_state_inertial_at_t0 = orbitsim::make_state({}, {});
        return arc;
    }

    Game::KeplerPredictionState make_prediction_with_active_track(
            const Game::KeplerOrbitArc &base_arc,
            std::string label = {},
            const Game::EntityId entity = Game::EntityId(7))
    {
        Game::KeplerPredictionState prediction{};
        prediction.valid = true;

        Game::KeplerPredictionState::Track track{};
        track.valid = true;
        track.active_player = true;
        track.entity = entity;
        track.label = std::move(label);
        track.primary_body_id = kEarthId;
        track.orbit.valid = true;
        track.orbit.status = Game::KeplerOrbitStatus::Ok;
        track.orbit.base_arc = base_arc;
        track.base_arcs.push_back(base_arc);
        track.world_frame.world_reference_body_world = WorldVec3{1'000.0, 2'000.0, 3'000.0};
        track.world_frame.world_reference_body_id = kEarthId;
        track.world_frame.world_reference_state_inertial = base_arc.primary_state_inertial_at_t0;
        track.body_state_provider.state_at =
                [primary = base_arc.primary_state_inertial_at_t0](
                        const orbitsim::BodyId body_id,
                        const double /*t_s*/,
                        orbitsim::State &out_state) {
                    if (body_id != kEarthId)
                    {
                        return false;
                    }
                    out_state = primary;
                    return true;
                };
        prediction.tracks.push_back(track);
        return prediction;
    }

    Game::KeplerManeuverOrbitPickInfo make_orbit_pick(const std::string_view owner_name,
                                                       const double time_s,
                                                       const bool line = true)
    {
        Game::KeplerManeuverOrbitPickInfo pick{};
        pick.valid = true;
        pick.line = line;
        pick.owner_name = owner_name;
        pick.time_s = time_s;
        return pick;
    }

    Game::KeplerManeuverOrbitPickInfo make_payload_orbit_pick(
            const Game::KeplerManeuverOrbitPickRole role,
            const Game::EntityId entity,
            const double time_s,
            const std::string_view owner_name = "KeplerOrbit/Base/Debug")
    {
        Game::KeplerManeuverOrbitPickInfo pick = make_orbit_pick(owner_name, time_s);
        pick.payload = Game::KeplerManeuverPick::make_payload(role, entity);
        return pick;
    }
} // namespace

TEST(KeplerManeuverSystem, AddSortSelectAndExposePredictionNodes)
{
    Game::KeplerManeuverSystem system{};

    const Game::KeplerManeuverCommandResult first =
            system.apply_command(Game::KeplerManeuverCommand::add_node(
                    make_node(4, 120.0, {0.0, 10.0, 0.0})));
    const Game::KeplerManeuverCommandResult second =
            system.apply_command(Game::KeplerManeuverCommand::add_node(
                    make_node(-1, 60.0, {1.0, 0.0, 0.0})));

    EXPECT_TRUE(first.applied);
    EXPECT_TRUE(second.applied);
    EXPECT_TRUE(second.prediction_dirty);
    EXPECT_EQ(second.added_node_id, 5);
    ASSERT_EQ(system.plan().nodes.size(), 2u);
    EXPECT_EQ(system.plan().nodes[0].id, 5);
    EXPECT_EQ(system.plan().nodes[1].id, 4);
    EXPECT_EQ(system.plan().selected_node_id, 5);
    EXPECT_EQ(system.revision(), 2u);

    const std::span<const Game::KeplerManeuverNode> nodes = system.prediction_nodes();
    ASSERT_EQ(nodes.size(), 2u);
    EXPECT_EQ(nodes[0].node_id, 5);
    EXPECT_DOUBLE_EQ(nodes[0].t_s, 60.0);
    EXPECT_EQ(nodes[0].primary_body_id, orbitsim::kInvalidBodyId);
    EXPECT_DOUBLE_EQ(nodes[0].dv_rtn_mps.x, 1.0);
    EXPECT_EQ(nodes[1].node_id, 4);
    EXPECT_DOUBLE_EQ(nodes[1].t_s, 120.0);
}

TEST(KeplerManeuverSystem, RejectsInvalidAddedNodes)
{
    Game::KeplerManeuverSystem system{};

    Game::KeplerManeuverEditorNode invalid_time = make_node(-1,
                                                            std::numeric_limits<double>::quiet_NaN());
    Game::KeplerManeuverEditorNode invalid_dv = make_node(-1, 30.0);
    invalid_dv.dv_rtn_mps.y = std::numeric_limits<double>::infinity();
    Game::KeplerManeuverEditorNode invalid_primary = make_node(-1, 40.0);
    invalid_primary.primary_body_auto = false;
    invalid_primary.primary_body_id = orbitsim::kInvalidBodyId;

    const Game::KeplerManeuverCommandResult time_result =
            system.apply_command(Game::KeplerManeuverCommand::add_node(invalid_time));
    const Game::KeplerManeuverCommandResult dv_result =
            system.apply_command(Game::KeplerManeuverCommand::add_node(invalid_dv));
    const Game::KeplerManeuverCommandResult primary_result =
            system.apply_command(Game::KeplerManeuverCommand::add_node(invalid_primary));

    EXPECT_FALSE(time_result.applied);
    EXPECT_FALSE(time_result.plan_changed);
    EXPECT_FALSE(time_result.prediction_dirty);
    EXPECT_EQ(time_result.added_node_id, -1);
    EXPECT_FALSE(dv_result.applied);
    EXPECT_FALSE(primary_result.applied);
    EXPECT_TRUE(system.plan().nodes.empty());
    EXPECT_EQ(system.plan().selected_node_id, -1);
    EXPECT_EQ(system.plan().next_node_id, 0);
    EXPECT_TRUE(system.prediction_nodes().empty());
    EXPECT_EQ(system.revision(), 0u);
}

TEST(KeplerManeuverSystem, SelectOnlyDoesNotChangeRevision)
{
    Game::KeplerManeuverSystem system{};
    ASSERT_TRUE(system.apply_command(Game::KeplerManeuverCommand::add_node(make_node(1, 10.0))).applied);
    ASSERT_TRUE(system.apply_command(Game::KeplerManeuverCommand::add_node(make_node(2, 20.0))).applied);
    const uint64_t revision = system.revision();

    const Game::KeplerManeuverCommandResult result =
            system.apply_command(Game::KeplerManeuverCommand::select_node(1));

    EXPECT_TRUE(result.applied);
    EXPECT_TRUE(result.selection_changed);
    EXPECT_FALSE(result.plan_changed);
    EXPECT_FALSE(result.prediction_dirty);
    EXPECT_EQ(system.plan().selected_node_id, 1);
    EXPECT_EQ(system.revision(), revision);
}

TEST(KeplerManeuverSystem, ConvertsExplicitAndAutoPrimaryBody)
{
    Game::KeplerManeuverSystem system{};
    Game::KeplerManeuverEditorNode node = make_node(1, 30.0, {0.0, 0.0, 2.0});
    node.primary_body_auto = false;
    node.primary_body_id = kEarthId;
    ASSERT_TRUE(system.apply_command(Game::KeplerManeuverCommand::add_node(node)).applied);

    std::span<const Game::KeplerManeuverNode> prediction_nodes = system.prediction_nodes();
    ASSERT_EQ(prediction_nodes.size(), 1u);
    EXPECT_EQ(prediction_nodes[0].primary_body_id, kEarthId);

    const Game::KeplerManeuverCommandResult result =
            system.apply_command(Game::KeplerManeuverCommand::set_node_primary_body(
                    1,
                    true,
                    kEarthId));

    EXPECT_TRUE(result.applied);
    EXPECT_TRUE(result.prediction_dirty);
    prediction_nodes = system.prediction_nodes();
    ASSERT_EQ(prediction_nodes.size(), 1u);
    EXPECT_EQ(prediction_nodes[0].primary_body_id, orbitsim::kInvalidBodyId);
    ASSERT_EQ(system.plan().nodes.size(), 1u);
    EXPECT_TRUE(system.plan().nodes.front().primary_body_auto);
    EXPECT_EQ(system.plan().nodes.front().primary_body_id, orbitsim::kInvalidBodyId);
}

TEST(KeplerManeuverSystem, RejectsInvalidExplicitPrimaryBodySetter)
{
    Game::KeplerManeuverSystem system{};
    ASSERT_TRUE(system.apply_command(Game::KeplerManeuverCommand::add_node(
            make_node(1, 30.0, {0.0, 0.0, 2.0}))).applied);
    const uint64_t revision = system.revision();

    const Game::KeplerManeuverCommandResult result =
            system.apply_command(Game::KeplerManeuverCommand::set_node_primary_body(
                    1,
                    false,
                    orbitsim::kInvalidBodyId));

    EXPECT_FALSE(result.applied);
    EXPECT_FALSE(result.plan_changed);
    EXPECT_FALSE(result.prediction_dirty);
    EXPECT_EQ(system.revision(), revision);
    ASSERT_EQ(system.plan().nodes.size(), 1u);
    EXPECT_TRUE(system.plan().nodes.front().primary_body_auto);
    EXPECT_EQ(system.plan().nodes.front().primary_body_id, orbitsim::kInvalidBodyId);
}

TEST(KeplerManeuverSystem, RemoveNodeClearsDependentInteraction)
{
    Game::KeplerManeuverSystem system{};
    ASSERT_TRUE(system.apply_command(Game::KeplerManeuverCommand::add_node(make_node(1, 10.0))).applied);
    ASSERT_TRUE(system.apply_command(Game::KeplerManeuverCommand::add_node(make_node(2, 20.0))).applied);
    system.interaction().state = Game::KeplerManeuverInteraction::State::DragAxis;
    system.interaction().node_id = 1;
    system.interaction().suppress_orbit_pick_until_left_release = true;

    const Game::KeplerManeuverCommandResult result =
            system.apply_command(Game::KeplerManeuverCommand::remove_node(1, 0));

    EXPECT_TRUE(result.applied);
    EXPECT_TRUE(result.nodes_removed);
    EXPECT_TRUE(result.prediction_dirty);
    EXPECT_EQ(system.plan().selected_node_id, 2);
    EXPECT_EQ(system.interaction().state, Game::KeplerManeuverInteraction::State::Idle);
    EXPECT_EQ(system.interaction().node_id, -1);
    EXPECT_TRUE(system.interaction().suppress_orbit_pick_until_left_release);

    const std::span<const Game::KeplerManeuverNode> nodes = system.prediction_nodes();
    ASSERT_EQ(nodes.size(), 1u);
    EXPECT_EQ(nodes[0].node_id, 2);
}

TEST(KeplerManeuverSystem, PrunesPastNodesAndUnlocksBaseOrbitCreation)
{
    Game::KeplerManeuverSystem system{};
    ASSERT_TRUE(system.apply_command(Game::KeplerManeuverCommand::add_node(make_node(1, 10.0))).applied);
    ASSERT_TRUE(system.apply_command(Game::KeplerManeuverCommand::add_node(make_node(2, 30.0))).applied);
    system.interaction().state = Game::KeplerManeuverInteraction::State::HoverHub;
    system.interaction().node_id = 1;

    const Game::KeplerManeuverCommandResult first =
            system.apply_command(Game::KeplerManeuverCommand::prune_past_nodes(10.0));

    EXPECT_TRUE(first.applied);
    EXPECT_TRUE(first.nodes_removed);
    EXPECT_TRUE(first.prediction_dirty);
    EXPECT_FALSE(first.plan_empty);
    ASSERT_EQ(first.removed_node_ids.size(), 1u);
    EXPECT_EQ(first.removed_node_ids.front(), 1);
    ASSERT_EQ(system.plan().nodes.size(), 1u);
    EXPECT_EQ(system.plan().nodes.front().id, 2);
    EXPECT_EQ(system.plan().selected_node_id, 2);
    EXPECT_EQ(system.interaction().state, Game::KeplerManeuverInteraction::State::Idle);
    ASSERT_EQ(system.prediction_nodes().size(), 1u);
    EXPECT_EQ(system.prediction_nodes().front().node_id, 2);

    const Game::KeplerManeuverCommandResult second =
            system.apply_command(Game::KeplerManeuverCommand::prune_past_nodes(31.0));

    EXPECT_TRUE(second.applied);
    EXPECT_TRUE(second.plan_empty);
    EXPECT_TRUE(system.plan().nodes.empty());
    EXPECT_EQ(system.plan().selected_node_id, -1);
    EXPECT_TRUE(system.prediction_nodes().empty());

    const orbitsim::State primary = orbitsim::make_state({}, {});
    const Game::KeplerPredictionState prediction =
            make_prediction_with_active_track(make_circular_arc(31.0, 120.0, primary), "Player");
    EXPECT_TRUE(Game::KeplerManeuverPick::can_create_node(
            make_orbit_pick("KeplerOrbit/Base/Player", 45.0),
            system.plan(),
            prediction,
            31.0));
}

TEST(KeplerManeuverSystem, DeferredDvDragWaitsForExplicitDirty)
{
    Game::KeplerManeuverSystem system{};
    ASSERT_TRUE(system.apply_command(Game::KeplerManeuverCommand::add_node(
            make_node(1, 10.0))).applied);
    const uint64_t after_add_revision = system.revision();
    ASSERT_EQ(system.prediction_nodes().size(), 1u);
    const orbitsim::Vec3 before_drag_dv = system.prediction_nodes().front().dv_rtn_mps;

    system.interaction().state = Game::KeplerManeuverInteraction::State::DragAxis;
    system.interaction().node_id = 1;

    const Game::KeplerManeuverCommandResult drag_result =
            system.apply_command(Game::KeplerManeuverCommand::set_node_dv(
                    1,
                    {0.0, 5.0, 0.0},
                    true));

    EXPECT_TRUE(drag_result.applied);
    EXPECT_TRUE(drag_result.plan_changed);
    EXPECT_FALSE(drag_result.prediction_dirty);
    EXPECT_EQ(system.revision(), after_add_revision);
    ASSERT_EQ(system.prediction_nodes().size(), 1u);
    EXPECT_EQ(system.prediction_nodes().front().dv_rtn_mps.x, before_drag_dv.x);
    EXPECT_EQ(system.prediction_nodes().front().dv_rtn_mps.y, before_drag_dv.y);
    EXPECT_EQ(system.prediction_nodes().front().dv_rtn_mps.z, before_drag_dv.z);
    EXPECT_TRUE(system.interaction().applied_delta);

    const Game::KeplerManeuverCommandResult dirty_result =
            system.apply_command(Game::KeplerManeuverCommand::mark_plan_dirty());

    EXPECT_TRUE(dirty_result.applied);
    EXPECT_TRUE(dirty_result.plan_changed);
    EXPECT_TRUE(dirty_result.prediction_dirty);
    EXPECT_EQ(system.revision(), after_add_revision + 1u);
    ASSERT_EQ(system.prediction_nodes().size(), 1u);
    EXPECT_EQ(system.prediction_nodes().front().dv_rtn_mps.y, 5.0);
}

TEST(KeplerManeuverSystem, DirtyDvDragPreservesDragDisplay)
{
    Game::KeplerManeuverSystem system{};
    ASSERT_TRUE(system.apply_command(Game::KeplerManeuverCommand::add_node(
            make_node(1, 10.0))).applied);

    const orbitsim::State primary = orbitsim::make_state({}, {});
    Game::KeplerPredictionState prediction =
            make_prediction_with_active_track(make_circular_arc(0.0, 120.0, primary));
    ASSERT_EQ(system.resolve_node_display_states(prediction).valid_node_count, 1u);
    ASSERT_EQ(system.node_display_states().size(), 1u);
    system.interaction().state = Game::KeplerManeuverInteraction::State::DragAxis;
    system.interaction().node_id = 1;

    const Game::KeplerManeuverCommandResult result =
            system.apply_command(Game::KeplerManeuverCommand::set_node_dv(
                    1,
                    {0.0, 4.0, 0.0}));

    EXPECT_TRUE(result.applied);
    EXPECT_TRUE(result.plan_changed);
    EXPECT_TRUE(result.prediction_dirty);
    EXPECT_EQ(system.interaction().state, Game::KeplerManeuverInteraction::State::DragAxis);
    EXPECT_EQ(system.interaction().node_id, 1);
    EXPECT_TRUE(system.interaction().applied_delta);
    ASSERT_EQ(system.node_display_states().size(), 1u);
    EXPECT_EQ(system.node_display_states().front().node_id, 1);
}

TEST(KeplerManeuverSystem, ResolvesNodeDisplayStateFromActiveKeplerTrack)
{
    Game::KeplerManeuverSystem system{};
    ASSERT_TRUE(system.apply_command(Game::KeplerManeuverCommand::add_node(
            make_node(1, 0.0, {0.0, 10.0, 0.0}))).applied);

    const orbitsim::State primary =
            orbitsim::make_state({10.0, 20.0, 30.0}, {0.0, 0.0, 0.0});
    Game::KeplerPredictionState prediction =
            make_prediction_with_active_track(make_circular_arc(0.0, 120.0, primary));

    const Game::KeplerManeuverNodeResolveResult result =
            system.resolve_node_display_states(prediction);

    EXPECT_TRUE(result.active_track_found);
    EXPECT_TRUE(result.active_track_valid);
    EXPECT_EQ(result.node_count, 1u);
    EXPECT_EQ(result.valid_node_count, 1u);
    ASSERT_EQ(system.node_display_states().size(), 1u);

    const Game::KeplerManeuverNodeDisplayState &display =
            system.node_display_states().front();
    EXPECT_TRUE(display.valid);
    EXPECT_TRUE(display.selected);
    EXPECT_EQ(display.status, Game::KeplerManeuverNodeDisplayStatus::Resolved);
    EXPECT_EQ(display.source, Game::KeplerManeuverNodeDisplaySource::BaseArc);
    EXPECT_EQ(display.primary_body_id, kEarthId);
    EXPECT_DOUBLE_EQ(display.position_world.x, 1'000.0 + kOrbitRadiusM);
    EXPECT_DOUBLE_EQ(display.position_world.y, 2'000.0);
    EXPECT_DOUBLE_EQ(display.position_world.z, 3'000.0);
    EXPECT_NEAR(display.basis_r_world.x, 1.0, 1.0e-12);
    EXPECT_NEAR(display.basis_t_world.y, 1.0, 1.0e-12);
    EXPECT_NEAR(display.basis_n_world.z, 1.0, 1.0e-12);
    EXPECT_NEAR(display.total_dv_mps, 10.0, 1.0e-12);
    EXPECT_NEAR(display.burn_direction_world.y, 1.0, 1.0e-12);
}

TEST(KeplerManeuverSystem, ResolvesHyperbolicNodeWithLibrarySamplingFallback)
{
    Game::KeplerManeuverSystem system{};
    const Game::KeplerOrbitArc arc = make_hyperbolic_arc();
    ASSERT_TRUE(system.apply_command(Game::KeplerManeuverCommand::add_node(
            make_node(1, arc.arc.t1_s, {0.0, 5.0, 0.0}))).applied);

    Game::KeplerPredictionState prediction = make_prediction_with_active_track(arc);
    prediction.tracks.front().line_propagation.max_hyperbolic_stumpff_arg = 0.2;

    const orbitsim::KeplerArcSample direct_sample =
            orbitsim::sample_kepler_arc_state(arc.arc,
                                              arc.arc.t1_s,
                                              prediction.tracks.front().line_propagation);
    ASSERT_TRUE(direct_sample.ok());
    EXPECT_TRUE(direct_sample.diagnostics.used_fallback);

    const Game::KeplerManeuverNodeResolveResult result =
            system.resolve_node_display_states(prediction);

    EXPECT_TRUE(result.active_track_valid);
    EXPECT_EQ(result.valid_node_count, 1u);
    ASSERT_EQ(system.node_display_states().size(), 1u);
    const Game::KeplerManeuverNodeDisplayState &display =
            system.node_display_states().front();
    EXPECT_TRUE(display.valid);
    EXPECT_EQ(display.status, Game::KeplerManeuverNodeDisplayStatus::Resolved);
    EXPECT_TRUE(std::isfinite(display.position_world.x));
    EXPECT_TRUE(std::isfinite(display.position_world.y));
    EXPECT_TRUE(std::isfinite(display.position_world.z));
}

TEST(KeplerManeuverSystem, UsesPlannedPreImpulseArcWhenAvailable)
{
    Game::KeplerManeuverSystem system{};
    ASSERT_TRUE(system.apply_command(Game::KeplerManeuverCommand::add_node(
            make_node(2, 60.0, {1.0, 0.0, 0.0}))).applied);

    const orbitsim::State primary = orbitsim::make_state({}, {});
    const Game::KeplerOrbitArc base_arc = make_circular_arc(0.0, 120.0, primary);
    Game::KeplerPredictionState prediction = make_prediction_with_active_track(base_arc);

    Game::KeplerOrbitArc pre_impulse = base_arc;
    pre_impulse.arc.t1_s = 60.0;
    Game::KeplerOrbitArc post_impulse = base_arc;
    post_impulse.arc.t0_s = 60.0;
    prediction.tracks.front().planned_arcs = {pre_impulse, post_impulse};

    const Game::KeplerManeuverNodeResolveResult result =
            system.resolve_node_display_states(prediction);

    ASSERT_EQ(result.valid_node_count, 1u);
    ASSERT_EQ(system.node_display_states().size(), 1u);
    EXPECT_TRUE(system.node_display_states().front().valid);
    EXPECT_EQ(system.node_display_states().front().source,
              Game::KeplerManeuverNodeDisplaySource::PlannedPreImpulseArc);
}

TEST(KeplerManeuverSystem, MarksNodeDisplayInvalidWithoutActiveTrack)
{
    Game::KeplerManeuverSystem system{};
    ASSERT_TRUE(system.apply_command(Game::KeplerManeuverCommand::add_node(
            make_node(3, 10.0))).applied);

    Game::KeplerPredictionState prediction{};
    const Game::KeplerManeuverNodeResolveResult result =
            system.resolve_node_display_states(prediction);

    EXPECT_FALSE(result.active_track_found);
    EXPECT_EQ(result.node_count, 1u);
    EXPECT_EQ(result.valid_node_count, 0u);
    ASSERT_EQ(system.node_display_states().size(), 1u);
    EXPECT_FALSE(system.node_display_states().front().valid);
    EXPECT_EQ(system.node_display_states().front().status,
              Game::KeplerManeuverNodeDisplayStatus::MissingActiveTrack);
}

TEST(KeplerManeuverOrbitPick, ParsesRoleQualifiedOwnerNames)
{
    std::string_view label{};

    EXPECT_EQ(Game::KeplerManeuverPick::parse_owner("KeplerOrbit/Base", &label),
              Game::KeplerManeuverOrbitPickRole::Base);
    EXPECT_TRUE(label.empty());

    EXPECT_EQ(Game::KeplerManeuverPick::parse_owner("KeplerOrbit/Planned/Player", &label),
              Game::KeplerManeuverOrbitPickRole::Planned);
    EXPECT_EQ(label, "Player");

    EXPECT_EQ(Game::KeplerManeuverPick::parse_owner("KeplerOrbit/Base/Player", &label),
              Game::KeplerManeuverOrbitPickRole::Base);
    EXPECT_EQ(label, "Player");

    EXPECT_EQ(Game::KeplerManeuverPick::parse_owner("OrbitPlot/Base", &label),
              Game::KeplerManeuverOrbitPickRole::None);
    EXPECT_TRUE(label.empty());
}

TEST(KeplerManeuverOrbitPick, UsesStructuredPayloadBeforeOwnerLabel)
{
    const Game::EntityId kPlayerEntity{77};
    const orbitsim::State primary = orbitsim::make_state({}, {});
    Game::KeplerPredictionState prediction =
            make_prediction_with_active_track(make_circular_arc(0.0, 120.0, primary),
                                              "Player",
                                              kPlayerEntity);
    Game::KeplerManeuverPlanState plan{};

    EXPECT_TRUE(Game::KeplerManeuverPick::can_create_node(
            make_payload_orbit_pick(Game::KeplerManeuverOrbitPickRole::Base,
                                    kPlayerEntity,
                                    30.0,
                                    "KeplerOrbit/Base/Other"),
            plan,
            prediction,
            10.0));
    EXPECT_FALSE(Game::KeplerManeuverPick::can_create_node(
            make_payload_orbit_pick(Game::KeplerManeuverOrbitPickRole::Base,
                                    Game::EntityId(78),
                                    30.0,
                                    "KeplerOrbit/Base/Player"),
            plan,
            prediction,
            10.0));
}

TEST(KeplerManeuverOrbitPick, AllowsBaseOnlyForEmptyPlanAndPlannedOnlyAfterNodesExist)
{
    const orbitsim::State primary = orbitsim::make_state({}, {});
    Game::KeplerPredictionState prediction =
            make_prediction_with_active_track(make_circular_arc(0.0, 120.0, primary), "Player");
    Game::KeplerManeuverPlanState plan{};

    EXPECT_TRUE(Game::KeplerManeuverPick::can_create_node(
            make_orbit_pick("KeplerOrbit/Base/Player", 30.0),
            plan,
            prediction,
            10.0));
    EXPECT_FALSE(Game::KeplerManeuverPick::can_create_node(
            make_orbit_pick("KeplerOrbit/Planned/Player", 30.0),
            plan,
            prediction,
            10.0));

    plan.nodes.push_back(make_node(1, 20.0));
    EXPECT_FALSE(Game::KeplerManeuverPick::can_create_node(
            make_orbit_pick("KeplerOrbit/Base/Player", 40.0),
            plan,
            prediction,
            10.0));
    EXPECT_TRUE(Game::KeplerManeuverPick::can_create_node(
            make_orbit_pick("KeplerOrbit/Planned/Player", 40.0),
            plan,
            prediction,
            10.0));
}

TEST(KeplerManeuverOrbitPick, RejectsInvalidTimeTrackAndKind)
{
    const orbitsim::State primary = orbitsim::make_state({}, {});
    Game::KeplerPredictionState prediction =
            make_prediction_with_active_track(make_circular_arc(0.0, 120.0, primary), "Player");
    Game::KeplerManeuverPlanState plan{};

    EXPECT_FALSE(Game::KeplerManeuverPick::can_create_node(
            make_orbit_pick("KeplerOrbit/Base/Player", 10.0),
            plan,
            prediction,
            10.0));
    EXPECT_FALSE(Game::KeplerManeuverPick::can_create_node(
            make_orbit_pick("KeplerOrbit/Base/Player", std::numeric_limits<double>::quiet_NaN()),
            plan,
            prediction,
            10.0));
    EXPECT_FALSE(Game::KeplerManeuverPick::can_create_node(
            make_orbit_pick("KeplerOrbit/Base/Other", 30.0),
            plan,
            prediction,
            10.0));
    EXPECT_FALSE(Game::KeplerManeuverPick::can_create_node(
            make_orbit_pick("KeplerOrbit/Base/Player",
                            30.0,
                            false),
            plan,
            prediction,
            10.0));
}

TEST(KeplerManeuverOrbitPick, CreatesAutoPrimaryZeroDvEditorNode)
{
    const orbitsim::State primary = orbitsim::make_state({}, {});
    Game::KeplerPredictionState prediction =
            make_prediction_with_active_track(make_circular_arc(0.0, 120.0, primary), "Player");
    const Game::KeplerManeuverOrbitPickInfo pick =
            make_orbit_pick("KeplerOrbit/Base/Player", 45.0);

    Game::KeplerManeuverSystem system{};
    ASSERT_TRUE(Game::KeplerManeuverPick::can_create_node(
            pick,
            system.plan(),
            prediction,
            10.0));

    const Game::KeplerManeuverCommandResult result =
            system.apply_command(Game::KeplerManeuverCommand::add_node(
                    Game::KeplerManeuverPick::make_node(pick)));

    ASSERT_TRUE(result.applied);
    ASSERT_EQ(system.plan().nodes.size(), 1u);
    const Game::KeplerManeuverEditorNode &node = system.plan().nodes.front();
    EXPECT_DOUBLE_EQ(node.time_s, 45.0);
    EXPECT_TRUE(node.primary_body_auto);
    EXPECT_EQ(node.primary_body_id, orbitsim::kInvalidBodyId);
    EXPECT_DOUBLE_EQ(node.dv_rtn_mps.x, 0.0);
    EXPECT_DOUBLE_EQ(node.dv_rtn_mps.y, 0.0);
    EXPECT_DOUBLE_EQ(node.dv_rtn_mps.z, 0.0);

    ASSERT_EQ(system.prediction_nodes().size(), 1u);
    EXPECT_EQ(system.prediction_nodes().front().primary_body_id, orbitsim::kInvalidBodyId);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
