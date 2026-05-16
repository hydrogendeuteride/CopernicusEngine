#include <gtest/gtest.h>

#include "game/orbit/kepler/kepler_celestial_nbody.h"
#include "game/orbit/kepler/kepler_arc_info.h"
#include "game/orbit/kepler/kepler_patched_conics_builder.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_builder.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_system.h"
#include "orbitsim/soi.hpp"

#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
    constexpr double kEarthMu = 3.986004418e14;
    constexpr orbitsim::BodyId kEarthId = 42;
    constexpr orbitsim::BodyId kMoonId = 43;

    bool near_abs(const double a, const double b, const double abs_tol)
    {
        return std::abs(a - b) <= abs_tol;
    }

    orbitsim::GameSimulation make_single_body_sim()
    {
        orbitsim::GameSimulation sim{};
        orbitsim::MassiveBody earth{};
        earth.id = kEarthId;
        earth.mass_kg = kEarthMu / sim.config().gravitational_constant;
        earth.radius_m = 6'371'000.0;
        earth.soi_radius_m = 9.0e8;
        earth.state = orbitsim::make_state({1'000.0, 2'000.0, 3'000.0}, {2.0, 0.0, 0.0});
        EXPECT_TRUE(sim.create_body_with_id(kEarthId, earth).valid());
        return sim;
    }

    orbitsim::CelestialEphemeris make_single_body_ephemeris(const orbitsim::GameSimulation &sim,
                                                            const double t0_s,
                                                            const double t1_s)
    {
        orbitsim::CelestialEphemeris ephemeris{};
        ephemeris.set_body_ids({kEarthId});

        orbitsim::CelestialEphemerisSegment segment{};
        segment.t0_s = t0_s;
        segment.dt_s = t1_s - t0_s;
        const orbitsim::MassiveBody *earth = sim.body_by_id(kEarthId);
        EXPECT_NE(earth, nullptr);
        const orbitsim::State state = earth ? earth->state : orbitsim::State{};
        segment.start.push_back(state);
        segment.end.push_back(state);
        ephemeris.segments.push_back(segment);
        return ephemeris;
    }

    orbitsim::State make_circular_ship_state(const orbitsim::State &primary_state)
    {
        constexpr double r_m = 7'000'000.0;
        const double v_mps = std::sqrt(kEarthMu / r_m);
        return orbitsim::make_state(primary_state.position_m + orbitsim::Vec3{r_m, 0.0, 0.0},
                                    primary_state.velocity_mps + orbitsim::Vec3{0.0, v_mps, 0.0});
    }

    Game::KeplerPredictionBuildRequest make_request(orbitsim::GameSimulation &sim)
    {
        const orbitsim::MassiveBody *earth = sim.body_by_id(kEarthId);
        EXPECT_NE(earth, nullptr);

        Game::KeplerPredictionBuildRequest request{};
        request.simulation = &sim;
        request.subject_state_inertial = make_circular_ship_state(earth->state);
        request.t0_s = 10.0;
        request.requested_horizon_s = 120.0;
        request.fixed_primary_body_id = kEarthId;
        request.world_frame.world_reference_body_world = WorldVec3{10'000.0, 0.0, 0.0};
        request.world_frame.world_reference_body_id = kEarthId;
        request.world_frame.world_reference_state_inertial = earth->state;
        request.body_state_provider.state_at =
                [&sim](const orbitsim::BodyId body_id, const double /*t_s*/, orbitsim::State &out_state) {
                    const orbitsim::MassiveBody *body = sim.body_by_id(body_id);
                    if (!body)
                    {
                        return false;
                    }
                    out_state = body->state;
                    return true;
                };
        request.line_options.max_time_step_s = 30.0;
        request.line_options.max_vertices_per_arc = 16;
        request.line_options.max_vertices_total = 32;
        return request;
    }

    Game::KeplerPredictionBuildRequest make_static_transfer_request(orbitsim::GameSimulation &sim)
    {
        const orbitsim::MassiveBody *earth = sim.body_by_id(kEarthId);
        EXPECT_NE(earth, nullptr);

        Game::KeplerPredictionBuildRequest request{};
        request.simulation = &sim;
        request.subject_state_inertial =
                orbitsim::make_state({20'000'000.0, 0.0, 0.0}, {7'000.0, 100.0, 0.0});
        request.t0_s = 0.0;
        request.requested_horizon_s = 30'000.0;
        request.fixed_primary_body_id = kEarthId;
        request.world_frame.world_reference_body_id = kEarthId;
        request.world_frame.world_reference_state_inertial = earth->state;
        request.body_state_provider.state_at =
                [&sim](const orbitsim::BodyId body_id, const double /*t_s*/, orbitsim::State &out_state) {
                    const orbitsim::MassiveBody *body = sim.body_by_id(body_id);
                    if (!body)
                    {
                        return false;
                    }
                    out_state = body->state;
                    return true;
                };
        request.options.patched_conics.max_search_step_s = 600.0;
        request.options.patched_conics.refine_tolerance_s = 0.5;
        request.line_options.max_time_step_s = 600.0;
        request.line_options.max_vertices_per_arc = 128;
        request.line_options.max_vertices_total = 256;
        return request;
    }

    orbitsim::GameSimulation make_static_transfer_sim()
    {
        orbitsim::GameSimulation sim{};

        orbitsim::MassiveBody earth{};
        earth.id = kEarthId;
        earth.mass_kg = kEarthMu / sim.config().gravitational_constant;
        earth.radius_m = 6'371'000.0;
        earth.soi_radius_m = 300'000'000.0;
        earth.state = orbitsim::make_state({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
        EXPECT_TRUE(sim.create_body_with_id(kEarthId, earth).valid());

        orbitsim::MassiveBody moon{};
        moon.id = kMoonId;
        moon.mass_kg = 7.342e22;
        moon.radius_m = 1'737'000.0;
        moon.soi_radius_m = 12'000'000.0;
        moon.state = orbitsim::make_state({100'000'000.0, 1'500'000.0, 0.0}, {0.0, 0.0, 0.0});
        EXPECT_TRUE(sim.create_body_with_id(kMoonId, moon).valid());

        return sim;
    }

    orbitsim::State sample_static_arc_inertial(const Game::KeplerOrbitArc &arc,
                                               const orbitsim::GameSimulation &sim,
                                               const double t_s)
    {
        const orbitsim::KeplerArcSample sample = orbitsim::sample_kepler_arc_state(arc.arc, t_s);
        EXPECT_TRUE(sample.ok());
        const orbitsim::MassiveBody *primary = sim.body_by_id(arc.arc.primary_body_id);
        EXPECT_NE(primary, nullptr);
        orbitsim::State out = sample.state_relative;
        out.position_m += primary->state.position_m;
        out.velocity_mps += primary->state.velocity_mps;
        return out;
    }

    orbitsim::GameSimulation make_two_body_sim()
    {
        orbitsim::GameSimulation sim{};
        const double g = sim.config().gravitational_constant;
        const double earth_mass_kg = kEarthMu / g;
        const double moon_mass_kg = 7.342e22;
        const double separation_m = 384'400'000.0;
        const double total_mass_kg = earth_mass_kg + moon_mass_kg;
        const double omega_radps = std::sqrt(g * total_mass_kg / (separation_m * separation_m * separation_m));
        const double earth_radius_m = separation_m * (moon_mass_kg / total_mass_kg);
        const double moon_radius_m = separation_m * (earth_mass_kg / total_mass_kg);

        orbitsim::MassiveBody earth{};
        earth.id = kEarthId;
        earth.mass_kg = earth_mass_kg;
        earth.radius_m = 6'371'000.0;
        earth.state = orbitsim::make_state({-earth_radius_m, 0.0, 0.0},
                                           {0.0, -omega_radps * earth_radius_m, 0.0});
        EXPECT_TRUE(sim.create_body_with_id(kEarthId, earth).valid());

        orbitsim::MassiveBody moon{};
        moon.id = kMoonId;
        moon.mass_kg = moon_mass_kg;
        moon.radius_m = 1'737'000.0;
        moon.state = orbitsim::make_state({moon_radius_m, 0.0, 0.0},
                                          {0.0, omega_radps * moon_radius_m, 0.0});
        EXPECT_TRUE(sim.create_body_with_id(kMoonId, moon).valid());

        return sim;
    }
} // namespace

TEST(KeplerPrediction, BuilderProducesBaseOrbitLinesAndMetrics)
{
    orbitsim::GameSimulation sim = make_single_body_sim();
    Game::KeplerPredictionBuildRequest request = make_request(sim);

    const Game::KeplerPredictionBuildOutput result = Game::build_kepler_prediction(request);

    ASSERT_TRUE(result.valid) << Game::kepler_orbit_status_name(result.status);
    ASSERT_TRUE(result.base_lines.valid);
    EXPECT_TRUE(result.planned_arcs.empty());
    EXPECT_EQ(result.base_arcs.size(), 1u);
    EXPECT_EQ(result.orbit.primary.body_id, kEarthId);
    EXPECT_TRUE(result.metrics.valid);
    EXPECT_EQ(result.metrics.primary_body_id, kEarthId);
    ASSERT_GE(result.base_lines.vertices.size(), 2u);
    EXPECT_TRUE(near_abs(result.base_lines.vertices.front().t_s, 10.0, 1.0e-12));
    EXPECT_TRUE(near_abs(result.base_lines.vertices.back().t_s, 130.0, 1.0e-12));
    EXPECT_TRUE(near_abs(result.base_lines.vertices.front().position_world.x,
                         7'010'000.0,
                         1.0e-6));
}

TEST(KeplerPrediction, PatchedConicsSplitsBaseOrbitAtMoonSoi)
{
    orbitsim::GameSimulation sim = make_static_transfer_sim();
    Game::KeplerPredictionBuildRequest request = make_static_transfer_request(sim);

    const Game::KeplerPredictionBuildOutput result = Game::build_kepler_prediction(request);

    ASSERT_TRUE(result.valid) << Game::kepler_orbit_status_name(result.status);
    ASSERT_GE(result.base_arcs.size(), 2u);
    EXPECT_EQ(result.base_arcs[0].arc.primary_body_id, kEarthId);
    EXPECT_EQ(result.base_arcs[1].arc.primary_body_id, kMoonId);
    ASSERT_FALSE(result.base_patch_events.empty());
    EXPECT_EQ(result.base_patch_events.front().reason, Game::KeplerPatchBoundaryReason::SoiTransition);
    EXPECT_EQ(result.base_patch_events.front().from_primary_body_id, kEarthId);
    EXPECT_EQ(result.base_patch_events.front().to_primary_body_id, kMoonId);
    EXPECT_TRUE(result.base_lines.valid);
}

TEST(KeplerPrediction, PatchedConicsKeepsTransitionStateContinuous)
{
    orbitsim::GameSimulation sim = make_static_transfer_sim();
    Game::KeplerPredictionBuildRequest request = make_static_transfer_request(sim);

    const Game::KeplerPredictionBuildOutput result = Game::build_kepler_prediction(request);

    ASSERT_TRUE(result.valid) << Game::kepler_orbit_status_name(result.status);
    ASSERT_GE(result.base_arcs.size(), 2u);
    const double transition_t_s = result.base_arcs[0].arc.t1_s;
    EXPECT_TRUE(near_abs(result.base_arcs[1].arc.t0_s, transition_t_s, 0.5));

    const orbitsim::State before = sample_static_arc_inertial(result.base_arcs[0], sim, transition_t_s);
    const orbitsim::State after = sample_static_arc_inertial(result.base_arcs[1], sim, result.base_arcs[1].arc.t0_s);
    EXPECT_LT(glm::length(before.position_m - after.position_m), 1.0e-3);
    EXPECT_LT(glm::length(before.velocity_mps - after.velocity_mps), 1.0e-6);
}

TEST(KeplerPrediction, SoiTransitionSearchSkipsWhenNoOtherSoiCandidateExists)
{
    orbitsim::GameSimulation sim = make_single_body_sim();
    const orbitsim::MassiveBody *earth = sim.body_by_id(kEarthId);
    ASSERT_NE(earth, nullptr);

    const orbitsim::KeplerArc arc{
            .mu_m3_s2 = kEarthMu,
            .primary_body_id = kEarthId,
            .t0_s = 0.0,
            .t1_s = 86'400.0,
            .state0_relative = orbitsim::make_state({7'000'000.0, 0.0, 0.0},
                                                    {0.0, std::sqrt(kEarthMu / 7'000'000.0), 0.0}),
    };

    const orbitsim::CelestialEphemeris ephemeris{};
    const orbitsim::SoiTransitionSearchResult transition =
            orbitsim::find_next_soi_transition_on_kepler_arc(sim,
                                                             ephemeris,
                                                             arc,
                                                             earth->id,
                                                             arc.t1_s);

    EXPECT_FALSE(transition.found);
    EXPECT_EQ(transition.first_failure, orbitsim::KeplerStatus::Ok);
    EXPECT_EQ(transition.tested_samples, 0u);
}

TEST(KeplerPrediction, SoiTransitionSearchReportsStepBudgetHit)
{
    orbitsim::GameSimulation sim = make_static_transfer_sim();

    const orbitsim::KeplerArc arc{
            .mu_m3_s2 = kEarthMu,
            .primary_body_id = kEarthId,
            .t0_s = 0.0,
            .t1_s = 1'000.0,
            .state0_relative = orbitsim::make_state({20'000'000.0, 0.0, 0.0},
                                                    {7'000.0, 100.0, 0.0}),
    };

    orbitsim::SoiTransitionSearchOptions options{};
    options.max_step_s = 10.0;
    options.max_steps = 1u;

    const orbitsim::CelestialEphemeris ephemeris{};
    const orbitsim::SoiTransitionSearchResult transition =
            orbitsim::find_next_soi_transition_on_kepler_arc(sim,
                                                             ephemeris,
                                                             arc,
                                                             kEarthId,
                                                             arc.t1_s,
                                                             options);

    EXPECT_FALSE(transition.found);
    EXPECT_TRUE(transition.budget_hit);
    EXPECT_EQ(transition.first_failure, orbitsim::KeplerStatus::Ok);
    EXPECT_EQ(transition.tested_samples, 1u);
    EXPECT_TRUE(near_abs(transition.last_tested_t_s, 10.0, 1.0e-12));
}

TEST(KeplerPrediction, PatchedConicsPatchLimitReportsTruncation)
{
    orbitsim::GameSimulation sim = make_static_transfer_sim();
    Game::KeplerPredictionBuildRequest request = make_static_transfer_request(sim);
    request.options.patched_conics.max_patches = 1u;

    const Game::KeplerPredictionBuildOutput result = Game::build_kepler_prediction(request);

    ASSERT_TRUE(result.valid) << Game::kepler_orbit_status_name(result.status);
    EXPECT_EQ(result.status, Game::KeplerOrbitStatus::SampleBudgetExceeded);
    ASSERT_EQ(result.base_arcs.size(), 1u);
    EXPECT_LT(result.base_arcs.back().arc.t1_s, request.t0_s + request.requested_horizon_s);
    EXPECT_TRUE(std::any_of(result.base_patch_events.begin(),
                            result.base_patch_events.end(),
                            [](const Game::KeplerPatchEvent &event) {
                                return event.reason == Game::KeplerPatchBoundaryReason::PatchLimit;
                            }));
}

TEST(KeplerPrediction, PatchedConicsPatchLimitCountsTinyPatchAttempts)
{
    orbitsim::GameSimulation sim = make_static_transfer_sim();
    Game::KeplerPredictionBuildRequest prediction_request = make_static_transfer_request(sim);
    prediction_request.options.patched_conics.max_patches = 1u;
    prediction_request.options.patched_conics.min_patch_duration_s = 20'000.0;

    Game::KeplerPatchChainBuildRequest request{};
    request.simulation = prediction_request.simulation;
    request.ephemeris = prediction_request.ephemeris;
    request.body_state_provider = prediction_request.body_state_provider;
    request.subject_state_inertial = prediction_request.subject_state_inertial;
    request.t0_s = prediction_request.t0_s;
    request.t1_s = prediction_request.t0_s + prediction_request.requested_horizon_s;
    request.current_primary_body_id = prediction_request.current_primary_body_id;
    request.fixed_initial_primary_body_id = prediction_request.fixed_primary_body_id;
    request.options = prediction_request.options;

    const Game::KeplerPatchChainBuildResult result =
            Game::build_kepler_patched_conics_chain(request);

    EXPECT_FALSE(result.valid);
    EXPECT_TRUE(result.arcs.empty());
    EXPECT_EQ(result.status, Game::KeplerOrbitStatus::SampleBudgetExceeded);
    EXPECT_TRUE(std::any_of(result.events.begin(),
                            result.events.end(),
                            [](const Game::KeplerPatchEvent &event) {
                                return event.reason == Game::KeplerPatchBoundaryReason::PatchLimit;
                            }));
}

TEST(KeplerPrediction, PatchedConicsSearchBudgetReportsTruncation)
{
    orbitsim::GameSimulation sim = make_static_transfer_sim();
    Game::KeplerPredictionBuildRequest request = make_static_transfer_request(sim);
    request.requested_horizon_s = 20.0;
    request.options.patched_conics.max_search_step_s = 0.001;

    const Game::KeplerPredictionBuildOutput result = Game::build_kepler_prediction(request);

    ASSERT_TRUE(result.valid) << Game::kepler_orbit_status_name(result.status);
    EXPECT_EQ(result.status, Game::KeplerOrbitStatus::SampleBudgetExceeded);
    ASSERT_EQ(result.base_arcs.size(), 1u);
    EXPECT_TRUE(near_abs(result.base_arcs.front().arc.t1_s, 10.0, 1.0e-9));
    EXPECT_TRUE(std::any_of(result.base_patch_events.begin(),
                            result.base_patch_events.end(),
                            [](const Game::KeplerPatchEvent &event) {
                                return event.reason == Game::KeplerPatchBoundaryReason::SearchBudget;
                            }));
}

TEST(KeplerPrediction, BuilderProducesPlannedLinesWhenNodesAreProvided)
{
    orbitsim::GameSimulation sim = make_single_body_sim();
    Game::KeplerPredictionBuildRequest request = make_request(sim);
    const std::vector<Game::KeplerManeuverNode> nodes{
            Game::KeplerManeuverNode{
                    .node_id = 7,
                    .t_s = 60.0,
                    .primary_body_id = kEarthId,
                    .dv_rtn_mps = {0.0, 10.0, 0.0},
            },
    };
    request.maneuver_nodes = std::span<const Game::KeplerManeuverNode>(nodes.data(), nodes.size());
    request.maneuver_revision = 3u;

    const Game::KeplerPredictionBuildOutput result = Game::build_kepler_prediction(request);

    ASSERT_TRUE(result.valid) << Game::kepler_orbit_status_name(result.status);
    EXPECT_EQ(result.maneuver_revision, 3u);
    EXPECT_TRUE(result.planned_requested);
    EXPECT_TRUE(result.planned_valid);
    EXPECT_EQ(result.planned_status, Game::KeplerOrbitStatus::Ok);
    ASSERT_EQ(result.planned_arcs.size(), 2u);
    ASSERT_TRUE(result.planned_lines.valid);
    ASSERT_GE(result.planned_lines.vertices.size(), 3u);
    EXPECT_TRUE(near_abs(result.planned_arcs[0].arc.t1_s, 60.0, 1.0e-12));
    EXPECT_TRUE(near_abs(result.planned_arcs[1].arc.t0_s, 60.0, 1.0e-12));
    EXPECT_GT(result.planned_lines.vertices.front().t_s, 59.0);
    EXPECT_LT(result.planned_lines.vertices.front().t_s, 61.0);
}

TEST(KeplerPrediction, BuilderKeepsLaterPlannedNodesBeyondBaseHorizon)
{
    orbitsim::GameSimulation sim = make_single_body_sim();
    Game::KeplerPredictionBuildRequest request = make_request(sim);
    const std::vector<Game::KeplerManeuverNode> nodes{
            Game::KeplerManeuverNode{
                    .node_id = 7,
                    .t_s = 60.0,
                    .primary_body_id = kEarthId,
                    .dv_rtn_mps = {0.0, 10.0, 0.0},
            },
            Game::KeplerManeuverNode{
                    .node_id = 8,
                    .t_s = 300.0,
                    .primary_body_id = kEarthId,
                    .dv_rtn_mps = {0.0, -5.0, 0.0},
            },
    };
    request.maneuver_nodes = std::span<const Game::KeplerManeuverNode>(nodes.data(), nodes.size());

    const Game::KeplerPredictionBuildOutput result = Game::build_kepler_prediction(request);

    ASSERT_TRUE(result.valid) << Game::kepler_orbit_status_name(result.status);
    EXPECT_TRUE(result.planned_requested);
    EXPECT_TRUE(result.planned_valid);
    ASSERT_EQ(result.planned_arcs.size(), 3u);
    EXPECT_TRUE(near_abs(result.planned_arcs[0].arc.t1_s, 60.0, 1.0e-12));
    EXPECT_TRUE(near_abs(result.planned_arcs[1].arc.t0_s, 60.0, 1.0e-12));
    EXPECT_TRUE(near_abs(result.planned_arcs[1].arc.t1_s, 300.0, 1.0e-12));
    EXPECT_TRUE(near_abs(result.planned_arcs[2].arc.t0_s, 300.0, 1.0e-12));
    ASSERT_TRUE(result.planned_lines.valid);
    EXPECT_GT(result.planned_lines.vertices.front().t_s, 59.0);
    EXPECT_LT(result.planned_lines.vertices.front().t_s, 61.0);
}

TEST(KeplerPrediction, BuilderKeepsBaseValidWhenPlannedArcFails)
{
    orbitsim::GameSimulation sim = make_single_body_sim();
    Game::KeplerPredictionBuildRequest request = make_request(sim);
    const std::vector<Game::KeplerManeuverNode> nodes{
            Game::KeplerManeuverNode{
                    .node_id = 7,
                    .t_s = 60.0,
                    .primary_body_id = kMoonId,
                    .dv_rtn_mps = {0.0, 10.0, 0.0},
            },
    };
    request.maneuver_nodes = std::span<const Game::KeplerManeuverNode>(nodes.data(), nodes.size());

    const Game::KeplerPredictionBuildOutput result = Game::build_kepler_prediction(request);

    ASSERT_TRUE(result.valid) << Game::kepler_orbit_status_name(result.status);
    EXPECT_EQ(result.status, Game::KeplerOrbitStatus::Ok);
    EXPECT_TRUE(result.base_lines.valid);
    EXPECT_TRUE(result.planned_requested);
    EXPECT_FALSE(result.planned_valid);
    EXPECT_EQ(result.planned_status, Game::KeplerOrbitStatus::PrimaryMismatch);
    EXPECT_TRUE(result.planned_arcs.empty());
    EXPECT_FALSE(result.planned_lines.valid);
}

TEST(KeplerPrediction, ManeuverNodeHorizonCoversLatestFutureNode)
{
    const std::vector<Game::KeplerManeuverNode> nodes{
            Game::KeplerManeuverNode{
                    .node_id = 1,
                    .t_s = 60.0,
                    .primary_body_id = kEarthId,
                    .dv_rtn_mps = {0.0, 10.0, 0.0},
            },
            Game::KeplerManeuverNode{
                    .node_id = 2,
                    .t_s = 300.0,
                    .primary_body_id = kEarthId,
                    .dv_rtn_mps = {0.0, -5.0, 0.0},
            },
            Game::KeplerManeuverNode{
                    .node_id = 3,
                    .t_s = std::numeric_limits<double>::quiet_NaN(),
                    .primary_body_id = kEarthId,
                    .dv_rtn_mps = {0.0, 1.0, 0.0},
            },
    };

    const double horizon_s =
            Game::required_kepler_maneuver_node_horizon_s(10.0,
                                                          nodes.data(),
                                                          nodes.size());
    EXPECT_TRUE(near_abs(horizon_s, 291.0, 1.0e-12));
    EXPECT_EQ(Game::required_kepler_maneuver_node_horizon_s(400.0,
                                                            nodes.data(),
                                                            nodes.size()),
              0.0);
    EXPECT_EQ(Game::required_kepler_maneuver_node_horizon_s(
                      std::numeric_limits<double>::quiet_NaN(),
                      nodes.data(),
                      nodes.size()),
              0.0);
    EXPECT_EQ(Game::required_kepler_maneuver_node_horizon_s(10.0, nullptr, 0u), 0.0);
}

TEST(KeplerPrediction, PlannedPreviewHorizonUsesPostBurnEllipticPeriod)
{
    orbitsim::GameSimulation sim = make_single_body_sim();
    Game::KeplerPredictionBuildRequest request = make_request(sim);
    request.requested_horizon_s = 120.0;
    request.options.open_orbit_window_s = 600.0;

    Game::KeplerArcBuildRequest orbit_request{};
    orbit_request.simulation = request.simulation;
    orbit_request.subject_state_inertial = request.subject_state_inertial;
    orbit_request.t0_s = request.t0_s;
    orbit_request.requested_horizon_s = request.requested_horizon_s;
    orbit_request.fixed_primary_body_id = request.fixed_primary_body_id;
    orbit_request.current_primary_body_id = request.current_primary_body_id;
    orbit_request.options = request.options;
    const Game::KeplerArcBuildResult orbit = Game::build_kepler_arc(orbit_request);
    ASSERT_TRUE(orbit.valid) << Game::kepler_orbit_status_name(orbit.status);

    const std::vector<Game::KeplerManeuverNode> nodes{
            Game::KeplerManeuverNode{
                    .node_id = 4,
                    .t_s = request.t0_s + 60.0,
                    .primary_body_id = kEarthId,
                    .dv_rtn_mps = {0.0, 1'500.0, 0.0},
            },
    };

    Game::KeplerOrbitArc maneuver_base = orbit.base_arc;
    maneuver_base.arc.t1_s = nodes.front().t_s + 1.0;
    const Game::KeplerManeuverArcBuildResult planned =
            Game::build_kepler_maneuver_arc_chain(maneuver_base,
                                                  std::span<const Game::KeplerManeuverNode>(
                                                          nodes.data(),
                                                          nodes.size()),
                                                  request.options.propagation);
    ASSERT_TRUE(planned.valid) << Game::kepler_orbit_status_name(planned.status);
    ASSERT_FALSE(planned.arcs.empty());
    const Game::KeplerArcMetrics metrics = Game::compute_kepler_arc_metrics(planned.arcs.back());
    ASSERT_TRUE(metrics.valid);
    EXPECT_GT(metrics.eccentricity, 0.35);

    const double horizon_s =
            Game::required_kepler_planned_preview_horizon_s(orbit,
                                                            nodes.data(),
                                                            nodes.size(),
                                                            request.options);

    EXPECT_GT(horizon_s,
              (nodes.front().t_s - request.t0_s) + request.options.open_orbit_window_s);
    EXPECT_TRUE(near_abs(horizon_s,
                         (nodes.front().t_s - request.t0_s) + metrics.period_s,
                         1.0e-6));
}

TEST(KeplerPrediction, PlannedPreviewEphemerisHorizonUsesBuiltPostBurnArc)
{
    orbitsim::GameSimulation sim = make_single_body_sim();
    Game::KeplerPredictionBuildRequest request = make_request(sim);
    request.options.patched_conics.enabled = true;
    request.requested_horizon_s = 120.0;
    request.options.open_orbit_window_s = 600.0;
    request.line_options.max_time_step_s = 300.0;
    request.line_options.max_vertices_per_arc = 32;
    request.line_options.max_vertices_total = 64;

    orbitsim::CelestialEphemeris ephemeris =
            make_single_body_ephemeris(sim, request.t0_s, request.t0_s + 1'000.0);
    request.ephemeris = &ephemeris;

    const std::vector<Game::KeplerManeuverNode> nodes{
            Game::KeplerManeuverNode{
                    .node_id = 5,
                    .t_s = request.t0_s + 60.0,
                    .primary_body_id = kEarthId,
                    .dv_rtn_mps = {0.0, 1'500.0, 0.0},
            },
    };
    request.maneuver_nodes = std::span<const Game::KeplerManeuverNode>(nodes.data(), nodes.size());

    const Game::KeplerPredictionBuildOutput result = Game::build_kepler_prediction(request);

    ASSERT_TRUE(result.valid) << Game::kepler_orbit_status_name(result.status);
    ASSERT_TRUE(result.planned_valid) << Game::kepler_orbit_status_name(result.planned_status);
    ASSERT_FALSE(result.planned_arcs.empty());
    ASSERT_FALSE(result.planned_patch_events.empty());
    EXPECT_LE(result.planned_arcs.back().arc.t1_s, ephemeris.t_end_s());

    const double horizon_s =
            Game::required_kepler_planned_preview_ephemeris_horizon_s(
                    std::span<const Game::KeplerOrbitArc>(result.planned_arcs.data(),
                                                          result.planned_arcs.size()),
                    std::span<const Game::KeplerPatchEvent>(result.planned_patch_events.data(),
                                                            result.planned_patch_events.size()),
                    request.t0_s,
                    ephemeris.t_end_s(),
                    request.options);

    const Game::KeplerOrbitArc &last_arc = result.planned_arcs.back();
    const double expected_horizon_s =
            last_arc.arc.t0_s +
            Game::select_kepler_arc_horizon_s(last_arc.arc, request.options) -
            request.t0_s;

    EXPECT_GT(horizon_s, ephemeris.t_end_s() - request.t0_s);
    EXPECT_TRUE(near_abs(horizon_s, expected_horizon_s, 1.0e-6));
}

TEST(KeplerPrediction, InputFingerprintTracksCacheRelevantSettings)
{
    Game::KeplerPredictionUpdateContext context{};
    context.requested_horizon_s = 120.0;
    context.fixed_primary_body_id = kEarthId;
    context.maneuver_revision = 4u;
    context.build_celestial_kepler_tracks = true;
    context.build_celestial_nbody_tracks = false;
    context.options.open_orbit_window_s = 3'600.0;
    context.options.propagation.max_iterations = 48;
    context.line_options.max_time_step_s = 30.0;
    context.line_options.max_vertices_total = 512u;

    Game::KeplerWorldFrame world_frame{};
    world_frame.world_reference_body_id = kEarthId;

    const Game::KeplerPredictionInputFingerprint baseline =
            Game::make_kepler_prediction_input_fingerprint(context, world_frame);
    Game::KeplerPredictionUpdateContext changed = context;
    changed.line_options.max_vertices_total += 1u;
    EXPECT_FALSE(baseline == Game::make_kepler_prediction_input_fingerprint(changed, world_frame));

    changed = context;
    changed.options.propagation.max_iterations += 1;
    EXPECT_FALSE(baseline == Game::make_kepler_prediction_input_fingerprint(changed, world_frame));

    changed = context;
    changed.build_celestial_nbody_tracks = true;
    EXPECT_FALSE(baseline == Game::make_kepler_prediction_input_fingerprint(changed, world_frame));

    changed = context;
    changed.options.patched_conics.max_patches += 1u;
    EXPECT_FALSE(baseline == Game::make_kepler_prediction_input_fingerprint(changed, world_frame));

    Game::KeplerWorldFrame changed_frame = world_frame;
    changed_frame.world_reference_body_id = kMoonId;
    EXPECT_FALSE(baseline == Game::make_kepler_prediction_input_fingerprint(context, changed_frame));
}

TEST(KeplerPrediction, BuildHorizonKeepsReusePaddingInsideCap)
{
    EXPECT_TRUE(near_abs(Game::kepler_prediction_build_horizon_s(3'600.0, 7'200.0),
                         4'500.0,
                         1.0e-12));
    EXPECT_TRUE(near_abs(Game::kepler_prediction_build_horizon_s(7'200.0, 7'200.0),
                         7'200.0,
                         1.0e-12));
    EXPECT_TRUE(near_abs(Game::kepler_prediction_build_horizon_s(7'000.0, 7'200.0),
                         7'200.0,
                         1.0e-12));
    EXPECT_TRUE(near_abs(Game::kepler_prediction_build_horizon_s(7'200.0, 0.0),
                         9'000.0,
                         1.0e-12));
}

TEST(KeplerPrediction, BuilderKeepsFullPlannedOpenOrbitLineWhenUniversalSolverRangeIsExceeded)
{
    orbitsim::GameSimulation sim = make_single_body_sim();
    Game::KeplerPredictionBuildRequest request = make_request(sim);
    request.options.open_orbit_window_s = 10'000.0;
    request.line_options.max_time_step_s = 60.0;
    request.line_options.max_chord_error_m = 0.0;
    request.line_options.max_vertices_per_arc = 64;
    request.line_options.max_vertices_total = 128;
    request.line_options.propagation.max_hyperbolic_stumpff_arg = 0.2;

    const std::vector<Game::KeplerManeuverNode> nodes{
            Game::KeplerManeuverNode{
                    .node_id = 7,
                    .t_s = 60.0,
                    .primary_body_id = kEarthId,
                    .dv_rtn_mps = {0.0, 5'000.0, 0.0},
            },
    };
    request.maneuver_nodes = std::span<const Game::KeplerManeuverNode>(nodes.data(), nodes.size());

    const Game::KeplerPredictionBuildOutput result = Game::build_kepler_prediction(request);

    ASSERT_TRUE(result.valid) << Game::kepler_orbit_status_name(result.status);
    ASSERT_TRUE(result.planned_lines.valid)
            << Game::kepler_orbit_status_name(result.planned_lines.diagnostics.status);
    ASSERT_GE(result.planned_lines.vertices.size(), 2u);
    EXPECT_EQ(result.planned_lines.diagnostics.status, Game::KeplerOrbitStatus::Ok);
    EXPECT_TRUE(near_abs(result.planned_lines.vertices.back().t_s,
                         result.planned_arcs.back().arc.t1_s,
                         1.0e-9));
    EXPECT_GT(result.planned_lines.vertices.back().t_s, nodes.front().t_s);
}

TEST(KeplerPrediction, CelestialNBodyEphemerisBuildsMovingBodyLines)
{
    orbitsim::GameSimulation sim = make_two_body_sim();
    const orbitsim::MassiveBody *earth = sim.body_by_id(kEarthId);
    const orbitsim::MassiveBody *moon = sim.body_by_id(kMoonId);
    ASSERT_NE(earth, nullptr);
    ASSERT_NE(moon, nullptr);

    Game::KeplerWorldFrame world_frame{};
    world_frame.world_reference_body_world = WorldVec3{10'000.0, 20'000.0, 30'000.0};
    world_frame.world_reference_body_id = kEarthId;
    world_frame.world_reference_state_inertial = earth->state;

    Game::KeplerPredictionOptions prediction_options{};
    Game::KeplerCelestialNBodyEphemerisResult ephemeris =
            Game::build_kepler_celestial_nbody_ephemeris(
                    Game::KeplerCelestialNBodyEphemerisRequest{
                            .simulation = &sim,
                            .world_frame = world_frame,
                            .t0_s = 0.0,
                            .requested_horizon_s = 3'600.0,
                            .options = prediction_options,
                    });

    ASSERT_TRUE(ephemeris.valid) << Game::kepler_orbit_status_name(ephemeris.status);
    ASSERT_TRUE(ephemeris.body_state_provider.state_at);

    orbitsim::State earth_t0{};
    orbitsim::State earth_t1{};
    ASSERT_TRUE(ephemeris.body_state_provider.state_at(kEarthId, 0.0, earth_t0));
    ASSERT_TRUE(ephemeris.body_state_provider.state_at(kEarthId, 3'600.0, earth_t1));
    EXPECT_GT(glm::length(earth_t1.position_m - earth_t0.position_m), 0.0);

    Game::KeplerArcLineOptions line_options{};
    line_options.max_time_step_s = 600.0;
    line_options.max_vertices_per_arc = 16u;
    line_options.max_vertices_total = 16u;

    const Game::KeplerArcLineSet moon_lines =
            Game::build_kepler_celestial_nbody_lines(
                    Game::KeplerCelestialNBodyLineRequest{
                            .ephemeris = ephemeris.ephemeris,
                            .body_id = kMoonId,
                            .world_frame = world_frame,
                            .t0_s = 0.0,
                            .requested_horizon_s = 3'600.0,
                            .line_options = line_options,
                    });

    ASSERT_TRUE(moon_lines.valid) << Game::kepler_orbit_status_name(moon_lines.diagnostics.status);
    ASSERT_GE(moon_lines.vertices.size(), 2u);
    const WorldVec3 expected_start =
            world_frame.world_reference_body_world +
            WorldVec3(moon->state.position_m - earth->state.position_m);
    EXPECT_TRUE(near_abs(moon_lines.vertices.front().position_world.x, expected_start.x, 1.0e-6));
    EXPECT_TRUE(near_abs(moon_lines.vertices.front().position_world.y, expected_start.y, 1.0e-6));
    EXPECT_TRUE(near_abs(moon_lines.vertices.front().position_world.z, expected_start.z, 1.0e-6));
    EXPECT_TRUE(near_abs(moon_lines.vertices.back().t_s, 3'600.0, 1.0e-12));
    EXPECT_GT(glm::length(glm::dvec3(moon_lines.vertices.back().position_world -
                                     moon_lines.vertices.front().position_world)),
              0.0);
}

TEST(KeplerPrediction, CelestialNBodyEphemerisUsesRequestedWindow)
{
    orbitsim::GameSimulation sim = make_two_body_sim();
    const orbitsim::MassiveBody *earth = sim.body_by_id(kEarthId);
    ASSERT_NE(earth, nullptr);

    Game::KeplerWorldFrame world_frame{};
    world_frame.world_reference_body_world = WorldVec3{0.0, 0.0, 0.0};
    world_frame.world_reference_body_id = kEarthId;
    world_frame.world_reference_state_inertial = earth->state;

    Game::KeplerPredictionOptions prediction_options{};
    prediction_options.open_orbit_window_s = 3'600.0;

    const Game::KeplerCelestialNBodyEphemerisResult ephemeris =
            Game::build_kepler_celestial_nbody_ephemeris(
                    Game::KeplerCelestialNBodyEphemerisRequest{
                            .simulation = &sim,
                            .world_frame = world_frame,
                            .t0_s = 0.0,
                            .requested_horizon_s = 7'200.0,
                            .options = prediction_options,
                    });

    ASSERT_TRUE(ephemeris.valid) << Game::kepler_orbit_status_name(ephemeris.status);
    EXPECT_TRUE(near_abs(ephemeris.horizon_s, 7'200.0, 1.0e-12));
    EXPECT_TRUE(near_abs(ephemeris.ephemeris->t_end_s(), 7'200.0, 1.0e-12));
}

TEST(KeplerPrediction, CelestialNBodyEphemerisFallsBackToPredictionWindow)
{
    orbitsim::GameSimulation sim = make_two_body_sim();
    const orbitsim::MassiveBody *earth = sim.body_by_id(kEarthId);
    ASSERT_NE(earth, nullptr);

    Game::KeplerWorldFrame world_frame{};
    world_frame.world_reference_body_world = WorldVec3{0.0, 0.0, 0.0};
    world_frame.world_reference_body_id = kEarthId;
    world_frame.world_reference_state_inertial = earth->state;

    Game::KeplerPredictionOptions prediction_options{};
    prediction_options.open_orbit_window_s = 7'200.0;

    const Game::KeplerCelestialNBodyEphemerisResult ephemeris =
            Game::build_kepler_celestial_nbody_ephemeris(
                    Game::KeplerCelestialNBodyEphemerisRequest{
                            .simulation = &sim,
                            .world_frame = world_frame,
                            .t0_s = 0.0,
                            .options = prediction_options,
                    });

    ASSERT_TRUE(ephemeris.valid) << Game::kepler_orbit_status_name(ephemeris.status);
    EXPECT_TRUE(near_abs(ephemeris.horizon_s, 7'200.0, 1.0e-12));
    EXPECT_TRUE(near_abs(ephemeris.ephemeris->t_end_s(), 7'200.0, 1.0e-12));
}

TEST(KeplerPrediction, CelestialNBodyHorizonLimitCapsLongWindows)
{
    Game::KeplerPredictionOptions prediction_options{};
    prediction_options.celestial_nbody_horizon_cap_s = 7'200.0;

    const Game::KeplerCelestialNBodyHorizonLimit capped =
            Game::limit_kepler_celestial_nbody_horizon(28'800.0,
                                                       prediction_options);
    EXPECT_TRUE(capped.capped);
    EXPECT_TRUE(near_abs(capped.uncapped_horizon_s, 28'800.0, 1.0e-12));
    EXPECT_TRUE(near_abs(capped.horizon_s, 7'200.0, 1.0e-12));
    EXPECT_TRUE(near_abs(capped.cap_s, 7'200.0, 1.0e-12));

    prediction_options.celestial_nbody_horizon_cap_s = 0.0;
    const Game::KeplerCelestialNBodyHorizonLimit uncapped =
            Game::limit_kepler_celestial_nbody_horizon(28'800.0,
                                                       prediction_options);
    EXPECT_FALSE(uncapped.capped);
    EXPECT_TRUE(near_abs(uncapped.horizon_s, 28'800.0, 1.0e-12));
    EXPECT_TRUE(near_abs(uncapped.cap_s, 0.0, 1.0e-12));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
