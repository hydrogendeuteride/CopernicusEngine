#include <gtest/gtest.h>

#include "game/orbit/kepler/kepler_celestial_nbody.h"
#include "game/orbit/kepler/kepler_arc_info.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_builder.h"

#include <glm/geometric.hpp>

#include <cmath>
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
    ASSERT_EQ(result.planned_arcs.size(), 2u);
    ASSERT_TRUE(result.planned_lines.valid);
    ASSERT_GE(result.planned_lines.vertices.size(), 3u);
    EXPECT_TRUE(near_abs(result.planned_arcs[0].arc.t1_s, 60.0, 1.0e-12));
    EXPECT_TRUE(near_abs(result.planned_arcs[1].arc.t0_s, 60.0, 1.0e-12));
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
