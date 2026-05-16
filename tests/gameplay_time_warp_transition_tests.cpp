#include "game/states/gameplay/gameplay_state.h"

#include <gtest/gtest.h>
#include <glm/geometric.hpp>

#include <cmath>
#include <memory>

namespace
{
    using Game::GameplayState;
    using Game::TimeWarpState;
    using Mode = Game::TimeWarpState::Mode;

    constexpr double kRuntimeEarthMu = 3.986004418e14;
    constexpr orbitsim::BodyId kRuntimeEarthId = 42;
    constexpr orbitsim::BodyId kRuntimeMoonId = 43;

    orbitsim::State sample_pure_earth_runtime_step(const orbitsim::State &spacecraft0,
                                                   const orbitsim::State &earth0,
                                                   const orbitsim::State &earth1,
                                                   const double t1_s)
    {
        const orbitsim::KeplerArc arc{
                .mu_m3_s2 = kRuntimeEarthMu,
                .primary_body_id = kRuntimeEarthId,
                .t0_s = 0.0,
                .t1_s = t1_s,
                .state0_relative = orbitsim::make_state(spacecraft0.position_m - earth0.position_m,
                                                        spacecraft0.velocity_mps - earth0.velocity_mps,
                                                        spacecraft0.spin),
        };

        const orbitsim::KeplerArcSample sample = orbitsim::sample_kepler_arc_state(arc, t1_s);
        EXPECT_TRUE(sample.ok());

        orbitsim::State out = sample.state_relative;
        out.position_m += earth1.position_m;
        out.velocity_mps += earth1.velocity_mps;
        return out;
    }
} // namespace

TEST(GameplayStateTimeWarpTransition, RailsRequestWithoutOrbitSimFallsBackToPhysicsWarp)
{
    GameplayState state{};
    Game::GameStateContext ctx{};

    state._orbit.scenario_owner().reset();
    state._orbital_physics.set_rails_warp_active_for_test(false);
    state._time_warp.mode = Mode::Realtime;
    state._time_warp.warp_level = 0;

    state.set_time_warp_level(ctx, TimeWarpState::kMaxPhysicsWarpLevel + 1);

    EXPECT_EQ(state._time_warp.warp_level, TimeWarpState::kMaxPhysicsWarpLevel);
    EXPECT_EQ(state._time_warp.mode, Mode::PhysicsWarp);
    EXPECT_FALSE(state._orbital_physics.rails_warp_active());
}

TEST(GameplayStateTimeWarpTransition, LeavingRailsWarpClearsRailsHandlesAndDisablesRailsState)
{
    GameplayState state{};
    Game::GameStateContext ctx{};

    Game::OrbiterInfo orbiter{};
    orbiter.entity = Game::EntityId{1};
    orbiter.rails.sc_id = 123;
    state._orbit.orbiters().push_back(orbiter);

    state._orbit.scenario_owner().reset();
    state._orbital_physics.set_rails_warp_active_for_test(true);
    state._time_warp.mode = Mode::RailsWarp;
    state._time_warp.warp_level = TimeWarpState::kMaxWarpLevel;

    state.set_time_warp_level(ctx, 0);

    ASSERT_EQ(state._orbit.orbiters().size(), 1u);
    EXPECT_EQ(state._time_warp.mode, Mode::Realtime);
    EXPECT_EQ(state._time_warp.warp_level, 0);
    EXPECT_FALSE(state._orbital_physics.rails_warp_active());
    EXPECT_EQ(state._orbit.orbiters()[0].rails.sc_id, orbitsim::kInvalidSpacecraftId);
}

TEST(GameplayStateTimeWarpTransition, WarpLevelClampsToBounds)
{
    GameplayState state{};
    Game::GameStateContext ctx{};

    state.set_time_warp_level(ctx, -999);
    EXPECT_EQ(state._time_warp.warp_level, 0);
    EXPECT_EQ(state._time_warp.mode, Mode::Realtime);

    // Keep mode in Rails so upper-bound clamp can be asserted without fallback.
    state._orbital_physics.set_rails_warp_active_for_test(true);
    state._time_warp.mode = Mode::RailsWarp;
    state._time_warp.warp_level = TimeWarpState::kMaxWarpLevel;

    state.set_time_warp_level(ctx, 999);
    EXPECT_EQ(state._time_warp.warp_level, TimeWarpState::kMaxWarpLevel);
    EXPECT_EQ(state._time_warp.mode, Mode::RailsWarp);
    EXPECT_TRUE(state._orbital_physics.rails_warp_active());
}

TEST(OrbitalPhysicsSystemRuntimeRails, DetectsTransientSoiFlythroughWithinOneStep)
{
    Game::GameWorld world{};
    Game::OrbitalRuntimeSystem orbit{};
    Game::ScenarioConfig scenario_config{};

    auto scenario = std::make_unique<Game::OrbitalScenario>();
    scenario->world_reference_body_index = 0u;

    orbitsim::MassiveBody earth{};
    earth.id = kRuntimeEarthId;
    earth.mass_kg = kRuntimeEarthMu / scenario->sim.config().gravitational_constant;
    earth.radius_m = 6'371'000.0;
    earth.soi_radius_m = 300'000'000.0;
    earth.state = orbitsim::make_state({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
    ASSERT_TRUE(scenario->sim.create_body_with_id(kRuntimeEarthId, earth).valid());
    scenario->bodies.push_back(Game::CelestialBodyInfo{
            .sim_id = kRuntimeEarthId,
            .name = "Earth",
            .radius_m = earth.radius_m,
            .mass_kg = earth.mass_kg,
    });

    orbitsim::MassiveBody moon{};
    moon.id = kRuntimeMoonId;
    moon.mass_kg = 7.342e22;
    moon.radius_m = 100'000.0;
    moon.soi_radius_m = 1'000'000.0;
    moon.state = orbitsim::make_state({30'000'000.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
    ASSERT_TRUE(scenario->sim.create_body_with_id(kRuntimeMoonId, moon).valid());
    scenario->bodies.push_back(Game::CelestialBodyInfo{
            .sim_id = kRuntimeMoonId,
            .name = "Moon",
            .radius_m = moon.radius_m,
            .mass_kg = moon.mass_kg,
    });

    orbitsim::Spacecraft spacecraft{};
    spacecraft.dry_mass_kg = 1'000.0;
    spacecraft.state = orbitsim::make_state({29'000'000.0, 500'000.0, 0.0},
                                            {40'000.0, 0.0, 0.0});
    const orbitsim::State spacecraft0 = spacecraft.state;
    const orbitsim::State earth0 = earth.state;
    const orbitsim::GameSimulation::SpacecraftHandle spacecraft_handle =
            scenario->sim.create_spacecraft(spacecraft);
    ASSERT_TRUE(spacecraft_handle.valid());

    Game::OrbiterInfo orbiter{};
    orbiter.name = "probe";
    orbiter.soi_kepler_primary_body_id = kRuntimeEarthId;
    orbiter.rails.sc_id = spacecraft_handle.id;

    orbit.set_scenario(std::move(scenario));
    orbit.orbiters().push_back(orbiter);

    Game::OrbitalPhysicsSystem physics{};
    Game::GameStateContext ctx{};
    orbitsim::SoiSwitchOptions switch_options{};
    switch_options.exit_scale = 1.0;

    Game::OrbitalPhysicsSystem::Context context{
            .renderer = nullptr,
            .world = world,
            .orbit = orbit,
            .physics = nullptr,
            .physics_context = nullptr,
            .scenario_config = scenario_config,
            .spacecraft_gravity_mode = Game::SpacecraftGravityMode::SoiKepler,
            .soi_switch_options = switch_options,
            .kepler_propagation = {},
            .soi_kepler_max_step_s = 60.0,
    };

    physics.rails_warp_step(context, ctx, 60.0);

    ASSERT_NE(orbit.scenario_owner(), nullptr);
    const orbitsim::Spacecraft *actual_spacecraft =
            orbit.scenario_owner()->sim.spacecraft_by_id(spacecraft_handle.id);
    ASSERT_NE(actual_spacecraft, nullptr);
    const orbitsim::MassiveBody *earth1 =
            orbit.scenario_owner()->sim.body_by_id(kRuntimeEarthId);
    ASSERT_NE(earth1, nullptr);

    const orbitsim::State pure_earth_end =
            sample_pure_earth_runtime_step(spacecraft0, earth0, earth1->state, 60.0);

    orbitsim::SoiSwitchOptions endpoint_probe_options = switch_options;
    endpoint_probe_options.fallback_to_max_accel = false;
    const orbitsim::CelestialEphemeris empty_ephemeris{};
    EXPECT_EQ(orbitsim::select_primary_body_id_rails(orbit.scenario_owner()->sim,
                                                     empty_ephemeris,
                                                     pure_earth_end.position_m,
                                                     60.0,
                                                     kRuntimeEarthId,
                                                     endpoint_probe_options),
              kRuntimeEarthId);

    EXPECT_EQ(orbit.orbiters().front().soi_kepler_primary_body_id, kRuntimeEarthId);
    EXPECT_GT(glm::length(actual_spacecraft->state.position_m - pure_earth_end.position_m), 10.0);
    EXPECT_GT(glm::length(actual_spacecraft->state.velocity_mps - pure_earth_end.velocity_mps), 1.0);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
