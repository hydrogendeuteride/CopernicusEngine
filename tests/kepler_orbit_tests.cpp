#include <gtest/gtest.h>

#include "game/orbit/kepler/kepler_debug.h"
#include "game/orbit/kepler/kepler_maneuver_solver.h"
#include "game/orbit/kepler/kepler_orbit_builder.h"
#include "game/orbit/kepler/kepler_orbit_metrics.h"
#include "game/orbit/kepler/kepler_orbit_tessellator.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace
{
    constexpr double kEarthMu = 3.986004418e14;
    constexpr orbitsim::BodyId kEarthId = 42;

    bool near_abs(const double a, const double b, const double abs_tol)
    {
        return std::abs(a - b) <= abs_tol;
    }

    bool near_vec_abs(const orbitsim::Vec3 &a, const orbitsim::Vec3 &b, const double abs_tol)
    {
        return near_abs(a.x, b.x, abs_tol) &&
               near_abs(a.y, b.y, abs_tol) &&
               near_abs(a.z, b.z, abs_tol);
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

    Game::KeplerOrbitArc make_circular_arc(const double t0_s = 0.0, const double t1_s = 1'000.0)
    {
        constexpr double r_m = 7'000'000.0;
        const double v_mps = std::sqrt(kEarthMu / r_m);

        Game::KeplerOrbitArc arc{};
        arc.arc.mu_m3_s2 = kEarthMu;
        arc.arc.primary_body_id = kEarthId;
        arc.arc.t0_s = t0_s;
        arc.arc.t1_s = t1_s;
        arc.arc.state0_relative = orbitsim::make_state({r_m, 0.0, 0.0}, {0.0, v_mps, 0.0});
        arc.primary_state_inertial_at_t0 =
                orbitsim::make_state({1'000.0, 0.0, 0.0}, {2.0, 0.0, 0.0});
        return arc;
    }
} // namespace

TEST(KeplerOrbit, BuilderSelectsFixedPrimaryAndBuildsRelativeArc)
{
    orbitsim::GameSimulation sim = make_single_body_sim();
    const orbitsim::MassiveBody *earth = sim.body_by_id(kEarthId);
    ASSERT_NE(earth, nullptr);

    constexpr double r_m = 7'000'000.0;
    const double v_mps = std::sqrt(kEarthMu / r_m);
    const orbitsim::State ship_state =
            orbitsim::make_state(earth->state.position_m + orbitsim::Vec3{r_m, 0.0, 0.0},
                                 earth->state.velocity_mps + orbitsim::Vec3{0.0, v_mps, 0.0});

    Game::KeplerOrbitBuildRequest request{};
    request.simulation = &sim;
    request.subject_state_inertial = ship_state;
    request.t0_s = 10.0;
    request.requested_horizon_s = 600.0;
    request.fixed_primary_body_id = kEarthId;

    const Game::KeplerOrbitBuildResult result = Game::build_kepler_orbit(request);

    ASSERT_TRUE(result.valid) << Game::kepler_orbit_status_name(result.status);
    EXPECT_EQ(result.primary.body_id, kEarthId);
    EXPECT_TRUE(near_abs(result.base_arc.arc.mu_m3_s2, kEarthMu, 1.0e-1));
    EXPECT_TRUE(near_vec_abs(result.base_arc.arc.state0_relative.position_m,
                             orbitsim::Vec3{r_m, 0.0, 0.0},
                             1.0e-9));
    EXPECT_TRUE(near_vec_abs(result.base_arc.arc.state0_relative.velocity_mps,
                             orbitsim::Vec3{0.0, v_mps, 0.0},
                             1.0e-12));
    EXPECT_TRUE(near_abs(result.base_arc.arc.t0_s, 10.0, 1.0e-12));
    EXPECT_TRUE(near_abs(result.base_arc.arc.t1_s, 610.0, 1.0e-12));
}

TEST(KeplerOrbit, BuilderHonorsExplicitHorizonWithoutMinimumClamp)
{
    orbitsim::GameSimulation sim = make_single_body_sim();
    const orbitsim::MassiveBody *earth = sim.body_by_id(kEarthId);
    ASSERT_NE(earth, nullptr);

    constexpr double r_m = 7'000'000.0;
    const double v_mps = std::sqrt(kEarthMu / r_m);
    const orbitsim::State ship_state =
            orbitsim::make_state(earth->state.position_m + orbitsim::Vec3{r_m, 0.0, 0.0},
                                 earth->state.velocity_mps + orbitsim::Vec3{0.0, v_mps, 0.0});

    Game::KeplerOrbitBuildRequest request{};
    request.simulation = &sim;
    request.subject_state_inertial = ship_state;
    request.t0_s = 10.0;
    request.requested_horizon_s = 1.0;
    request.fixed_primary_body_id = kEarthId;

    const Game::KeplerOrbitBuildResult result = Game::build_kepler_orbit(request);

    ASSERT_TRUE(result.valid) << Game::kepler_orbit_status_name(result.status);
    EXPECT_TRUE(near_abs(result.horizon_s, 1.0, 1.0e-12));
    EXPECT_TRUE(near_abs(result.base_arc.arc.t1_s, 11.0, 1.0e-12));
}

TEST(KeplerOrbit, BuilderUsesOpenOrbitWindowForHyperbolicArc)
{
    orbitsim::GameSimulation sim = make_single_body_sim();
    const orbitsim::MassiveBody *earth = sim.body_by_id(kEarthId);
    ASSERT_NE(earth, nullptr);

    constexpr double r_m = 7'000'000.0;
    const orbitsim::State ship_state =
            orbitsim::make_state(earth->state.position_m + orbitsim::Vec3{r_m, 0.0, 0.0},
                                 earth->state.velocity_mps + orbitsim::Vec3{0.0, 12'000.0, 0.0});

    Game::KeplerOrbitBuildRequest request{};
    request.simulation = &sim;
    request.subject_state_inertial = ship_state;
    request.t0_s = 0.0;
    request.fixed_primary_body_id = kEarthId;
    request.options.open_orbit_window_s = 45.0 * 24.0 * 60.0 * 60.0;

    const Game::KeplerOrbitBuildResult result = Game::build_kepler_orbit(request);

    ASSERT_TRUE(result.valid) << Game::kepler_orbit_status_name(result.status);
    EXPECT_TRUE(near_abs(result.horizon_s, request.options.open_orbit_window_s, 1.0e-12));
}

TEST(KeplerOrbit, PrimaryResolverSelectsLocalSoiBody)
{
    orbitsim::GameSimulation sim{};

    orbitsim::MassiveBody planet{};
    planet.id = 1;
    planet.mass_kg = 1.0e24;
    planet.soi_radius_m = 1.0e9;
    planet.state = orbitsim::make_state({0.0, 0.0, 0.0}, {});
    ASSERT_TRUE(sim.create_body_with_id(planet.id, planet).valid());

    orbitsim::MassiveBody moon{};
    moon.id = 2;
    moon.mass_kg = 1.0e22;
    moon.soi_radius_m = 1.0e6;
    moon.state = orbitsim::make_state({10'000'000.0, 0.0, 0.0}, {});
    ASSERT_TRUE(sim.create_body_with_id(moon.id, moon).valid());

    Game::KeplerPrimaryResolveRequest request{};
    request.simulation = &sim;
    request.query_position_m = {10'100'000.0, 0.0, 0.0};
    request.query_time_s = 0.0;
    request.current_primary_body_id = planet.id;

    const Game::KeplerPrimaryResolution result = Game::resolve_kepler_primary(request);

    ASSERT_TRUE(result.valid) << Game::kepler_orbit_status_name(result.status);
    EXPECT_EQ(result.body_id, moon.id);
}

TEST(KeplerOrbit, PrimaryResolverKeepsCurrentPrimaryDuringMaxAccelFallbackHysteresis)
{
    orbitsim::GameSimulation sim{};

    orbitsim::MassiveBody current{};
    current.id = 1;
    current.mass_kg = 1.0e20;
    current.soi_radius_m = 0.0;
    current.state = orbitsim::make_state({10.0, 0.0, 0.0}, {});
    ASSERT_TRUE(sim.create_body_with_id(current.id, current).valid());

    orbitsim::MassiveBody challenger{};
    challenger.id = 2;
    challenger.mass_kg = 1.0e20;
    challenger.soi_radius_m = 0.0;
    challenger.state = orbitsim::make_state({-9.8, 0.0, 0.0}, {});
    ASSERT_TRUE(sim.create_body_with_id(challenger.id, challenger).valid());

    Game::KeplerPrimaryResolveRequest request{};
    request.simulation = &sim;
    request.query_position_m = {0.0, 0.0, 0.0};
    request.query_time_s = 0.0;
    request.current_primary_body_id = current.id;
    request.fallback_primary_hysteresis_keep_ratio = 0.90;

    const Game::KeplerPrimaryResolution kept = Game::resolve_kepler_primary(request);
    ASSERT_TRUE(kept.valid) << Game::kepler_orbit_status_name(kept.status);
    EXPECT_EQ(kept.body_id, current.id);

    ASSERT_TRUE(sim.set_body_state(challenger.id, orbitsim::make_state({-8.0, 0.0, 0.0}, {})));
    const Game::KeplerPrimaryResolution switched = Game::resolve_kepler_primary(request);
    ASSERT_TRUE(switched.valid) << Game::kepler_orbit_status_name(switched.status);
    EXPECT_EQ(switched.body_id, challenger.id);
}

TEST(KeplerOrbit, ManeuverSolverSplitsArcAtTangentialImpulse)
{
    const Game::KeplerOrbitArc base = make_circular_arc(0.0, 1'000.0);
    const std::vector<Game::KeplerManeuverNode> nodes{
            Game::KeplerManeuverNode{
                    .node_id = 7,
                    .t_s = 100.0,
                    .primary_body_id = kEarthId,
                    .dv_rtn_mps = {0.0, 10.0, 0.0},
            },
    };

    const Game::KeplerManeuverSolveResult result =
            Game::build_kepler_maneuver_arcs(base, nodes);

    ASSERT_TRUE(result.valid) << Game::kepler_orbit_status_name(result.status);
    ASSERT_EQ(result.arcs.size(), 2u);
    EXPECT_TRUE(near_abs(result.arcs[0].arc.t0_s, 0.0, 1.0e-12));
    EXPECT_TRUE(near_abs(result.arcs[0].arc.t1_s, 100.0, 1.0e-12));
    EXPECT_TRUE(near_abs(result.arcs[1].arc.t0_s, 100.0, 1.0e-12));
    EXPECT_TRUE(near_abs(result.arcs[1].arc.t1_s, 1'000.0, 1.0e-12));

    const orbitsim::KeplerArcSample pre_impulse =
            orbitsim::sample_kepler_arc_state(base.arc, 100.0);
    ASSERT_TRUE(pre_impulse.ok());
    const orbitsim::State expected_post =
            orbitsim::apply_impulse_rtn(pre_impulse.state_relative, {0.0, 10.0, 0.0});
    EXPECT_TRUE(near_vec_abs(result.arcs[1].arc.state0_relative.position_m,
                             expected_post.position_m,
                             1.0e-6));
    EXPECT_TRUE(near_vec_abs(result.arcs[1].arc.state0_relative.velocity_mps,
                             expected_post.velocity_mps,
                             1.0e-9));
}

TEST(KeplerOrbit, ManeuverSolverRejectsCrossPrimaryImpulseForNow)
{
    const Game::KeplerOrbitArc base = make_circular_arc(0.0, 1'000.0);
    const std::vector<Game::KeplerManeuverNode> nodes{
            Game::KeplerManeuverNode{
                    .node_id = 8,
                    .t_s = 100.0,
                    .primary_body_id = 123,
                    .dv_rtn_mps = {0.0, 10.0, 0.0},
            },
    };

    const Game::KeplerManeuverSolveResult result =
            Game::build_kepler_maneuver_arcs(base, nodes);

    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.status, Game::KeplerOrbitStatus::PrimaryMismatch);
}

TEST(KeplerOrbit, TessellatorPreservesEndpointsAndSkipsDuplicateArcBoundary)
{
    const Game::KeplerOrbitArc base = make_circular_arc(0.0, 100.0);
    const std::vector<Game::KeplerManeuverNode> nodes{
            Game::KeplerManeuverNode{
                    .node_id = 9,
                    .t_s = 50.0,
                    .primary_body_id = kEarthId,
                    .dv_rtn_mps = {0.0, 5.0, 0.0},
            },
    };
    const Game::KeplerManeuverSolveResult solved = Game::build_kepler_maneuver_arcs(base, nodes);
    ASSERT_TRUE(solved.valid);

    Game::KeplerOrbitTessellationRequest request{};
    request.arcs = solved.arcs;
    request.options.max_time_step_s = 50.0;
    request.options.max_vertices_per_arc = 8;
    request.options.max_vertices_total = 16;
    request.world_frame.world_reference_state_inertial = {};

    const Game::KeplerOrbitLineSet lines = Game::build_kepler_orbit_lines(request);

    ASSERT_TRUE(lines.valid) << Game::kepler_orbit_status_name(lines.diagnostics.status);
    ASSERT_EQ(lines.vertices.size(), 3u);
    EXPECT_TRUE(near_abs(lines.vertices[0].t_s, 0.0, 1.0e-12));
    EXPECT_TRUE(near_abs(lines.vertices[1].t_s, 50.0, 1.0e-12));
    EXPECT_TRUE(near_abs(lines.vertices[2].t_s, 100.0, 1.0e-12));
    EXPECT_NE(lines.vertices.front().flags & static_cast<uint32_t>(Game::KeplerOrbitLineVertexFlags::OrbitStart), 0u);
    EXPECT_NE(lines.vertices.back().flags & static_cast<uint32_t>(Game::KeplerOrbitLineVertexFlags::OrbitEnd), 0u);
    EXPECT_NE(lines.vertices[1].flags & static_cast<uint32_t>(Game::KeplerOrbitLineVertexFlags::ArcEnd), 0u);
    EXPECT_NE(lines.vertices[1].flags & static_cast<uint32_t>(Game::KeplerOrbitLineVertexFlags::ArcStart), 0u);
}

TEST(KeplerOrbit, TessellatorAppliesMovingPrimaryProvider)
{
    const Game::KeplerOrbitArc arc = make_circular_arc(0.0, 20.0);

    Game::KeplerOrbitTessellationRequest fallback_request{};
    fallback_request.arcs = std::span<const Game::KeplerOrbitArc>(&arc, 1u);
    fallback_request.options.max_time_step_s = 10.0;
    fallback_request.options.max_vertices_per_arc = 4;
    fallback_request.options.max_vertices_total = 4;

    const Game::KeplerOrbitLineSet fallback_lines = Game::build_kepler_orbit_lines(fallback_request);
    ASSERT_TRUE(fallback_lines.valid);
    ASSERT_EQ(fallback_lines.vertices.size(), 3u);

    Game::KeplerOrbitTessellationRequest moving_request = fallback_request;
    moving_request.body_state_provider.state_at =
            [](const orbitsim::BodyId body_id, const double t_s, orbitsim::State &out_state) {
                if (body_id != kEarthId)
                {
                    return false;
                }
                out_state = orbitsim::make_state({1'000.0 + 2.0 * t_s, 0.0, 0.0},
                                                 {2.0, 0.0, 0.0});
                return true;
            };

    const Game::KeplerOrbitLineSet moving_lines = Game::build_kepler_orbit_lines(moving_request);
    ASSERT_TRUE(moving_lines.valid);
    ASSERT_EQ(moving_lines.vertices.size(), fallback_lines.vertices.size());

    const double primary_motion_x =
            moving_lines.vertices.back().position_world.x -
            fallback_lines.vertices.back().position_world.x;
    EXPECT_TRUE(near_abs(primary_motion_x, 40.0, 1.0e-9));
}

TEST(KeplerOrbit, MetricsClassifyCircularOrbit)
{
    const Game::KeplerOrbitArc arc = make_circular_arc();
    const Game::KeplerOrbitMetrics metrics = Game::compute_kepler_orbit_metrics(arc);

    ASSERT_TRUE(metrics.valid);
    EXPECT_EQ(metrics.primary_body_id, kEarthId);
    EXPECT_EQ(metrics.regime, orbitsim::KeplerOrbitRegime::Elliptic);
    EXPECT_TRUE(near_abs(metrics.eccentricity, 0.0, 1.0e-9));
    EXPECT_TRUE(near_abs(metrics.periapsis_radius_m, 7'000'000.0, 1.0e-3));
    EXPECT_TRUE(metrics.has_apoapsis);
    EXPECT_TRUE(near_abs(metrics.apoapsis_radius_m, 7'000'000.0, 1.0e-3));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
