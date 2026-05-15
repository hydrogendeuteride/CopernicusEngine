#include <gtest/gtest.h>

#include "game/orbit/kepler/kepler_celestial_nbody.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_builder.h"

#include <algorithm>
#include <span>
#include <vector>

namespace
{
    constexpr double kEarthMu = 3.986004418e14;
    constexpr orbitsim::BodyId kEarthId = 42;
    constexpr orbitsim::BodyId kMoonId = 43;

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

    Game::KeplerPredictionBuildRequest make_short_planned_transfer_request(orbitsim::GameSimulation &sim)
    {
        const orbitsim::MassiveBody *earth = sim.body_by_id(kEarthId);
        EXPECT_NE(earth, nullptr);

        Game::KeplerPredictionBuildRequest request{};
        request.simulation = &sim;
        request.subject_state_inertial =
                orbitsim::make_state({20'000'000.0, 0.0, 0.0}, {7'000.0, 100.0, 0.0});
        request.t0_s = 0.0;
        request.requested_horizon_s = 120.0;
        request.fixed_primary_body_id = kEarthId;
        request.world_frame.world_reference_body_id = kEarthId;
        request.world_frame.world_reference_state_inertial = earth->state;
        request.body_state_provider.state_at =
                [&sim](const orbitsim::BodyId body_id, const double, orbitsim::State &out_state) {
                    const orbitsim::MassiveBody *body = sim.body_by_id(body_id);
                    if (!body)
                    {
                        return false;
                    }
                    out_state = body->state;
                    return true;
                };
        request.options.open_orbit_window_s = 30'000.0;
        request.options.patched_conics.max_search_step_s = 600.0;
        request.options.patched_conics.refine_tolerance_s = 0.5;
        request.line_options.max_time_step_s = 600.0;
        request.line_options.max_vertices_per_arc = 128;
        request.line_options.max_vertices_total = 256;
        return request;
    }

    orbitsim::CelestialEphemeris make_static_ephemeris(const orbitsim::GameSimulation &sim,
                                                       const double t0_s,
                                                       const double t1_s)
    {
        orbitsim::CelestialEphemeris ephemeris{};
        ephemeris.set_body_ids({kEarthId, kMoonId});

        orbitsim::CelestialEphemerisSegment segment{};
        segment.t0_s = t0_s;
        segment.dt_s = t1_s - t0_s;
        segment.start.reserve(ephemeris.body_ids.size());
        segment.end.reserve(ephemeris.body_ids.size());
        for (const orbitsim::BodyId body_id : ephemeris.body_ids)
        {
            const orbitsim::MassiveBody *body = sim.body_by_id(body_id);
            EXPECT_NE(body, nullptr);
            const orbitsim::State state = body ? body->state : orbitsim::State{};
            segment.start.push_back(state);
            segment.end.push_back(state);
        }
        ephemeris.segments.push_back(segment);
        return ephemeris;
    }
} // namespace

TEST(KeplerPrediction, PatchedConicsSplitsPlannedPreviewBeyondBaseHorizon)
{
    orbitsim::GameSimulation sim = make_static_transfer_sim();
    Game::KeplerPredictionBuildRequest request = make_short_planned_transfer_request(sim);
    const std::vector<Game::KeplerManeuverNode> nodes{
            Game::KeplerManeuverNode{
                    .node_id = 11,
                    .t_s = 60.0,
                    .primary_body_id = kEarthId,
                    .dv_rtn_mps = {0.0, 0.0, 0.0},
            },
    };
    request.maneuver_nodes = std::span<const Game::KeplerManeuverNode>(nodes.data(), nodes.size());

    const Game::KeplerPredictionBuildOutput result = Game::build_kepler_prediction(request);

    ASSERT_TRUE(result.valid) << Game::kepler_orbit_status_name(result.status);
    ASSERT_TRUE(result.planned_valid) << Game::kepler_orbit_status_name(result.planned_status);
    ASSERT_TRUE(result.planned_lines.valid);
    ASSERT_EQ(result.base_arcs.size(), 1u);
    ASSERT_GE(result.planned_arcs.size(), 3u);
    EXPECT_TRUE(std::any_of(result.planned_arcs.begin(),
                            result.planned_arcs.end(),
                            [](const Game::KeplerOrbitArc &arc) {
                                return arc.arc.primary_body_id == kMoonId;
                            }));
    EXPECT_TRUE(std::any_of(result.planned_patch_events.begin(),
                            result.planned_patch_events.end(),
                            [](const Game::KeplerPatchEvent &event) {
                                return event.reason == Game::KeplerPatchBoundaryReason::SoiTransition &&
                                       event.to_primary_body_id == kMoonId;
                            }));
}

TEST(KeplerPrediction, CelestialNBodyHorizonCapKeepsPatchedConicsFloor)
{
    Game::KeplerPredictionOptions options{};
    options.celestial_nbody_horizon_cap_s = 7'200.0;

    const Game::KeplerCelestialNBodyHorizonLimit floor_kept =
            Game::limit_kepler_celestial_nbody_horizon(28'800.0, options, 28'800.0);
    EXPECT_FALSE(floor_kept.capped);
    EXPECT_EQ(floor_kept.cap_s, 7'200.0);
    EXPECT_EQ(floor_kept.horizon_s, 28'800.0);

    const Game::KeplerCelestialNBodyHorizonLimit capped_to_floor =
            Game::limit_kepler_celestial_nbody_horizon(57'600.0, options, 28'800.0);
    EXPECT_TRUE(capped_to_floor.capped);
    EXPECT_EQ(capped_to_floor.cap_s, 7'200.0);
    EXPECT_EQ(capped_to_floor.horizon_s, 28'800.0);
}

TEST(KeplerPrediction, PatchedConicsAppliesManeuverAtPredictionStart)
{
    orbitsim::GameSimulation sim = make_static_transfer_sim();
    Game::KeplerPredictionBuildRequest request = make_short_planned_transfer_request(sim);
    const std::vector<Game::KeplerManeuverNode> nodes{
            Game::KeplerManeuverNode{
                    .node_id = 12,
                    .t_s = request.t0_s,
                    .primary_body_id = kEarthId,
                    .dv_rtn_mps = {0.0, 50.0, 0.0},
            },
    };
    request.maneuver_nodes = std::span<const Game::KeplerManeuverNode>(nodes.data(), nodes.size());

    const Game::KeplerPredictionBuildOutput result = Game::build_kepler_prediction(request);

    ASSERT_TRUE(result.valid) << Game::kepler_orbit_status_name(result.status);
    ASSERT_TRUE(result.planned_valid) << Game::kepler_orbit_status_name(result.planned_status);
    ASSERT_FALSE(result.planned_arcs.empty());
    EXPECT_EQ(result.planned_diagnostics.impulses_applied, 1u);
    EXPECT_EQ(result.planned_arcs.front().arc.t0_s, request.t0_s);
    EXPECT_GT(glm::length(result.planned_arcs.front().arc.state0_relative.velocity_mps -
                          result.base_arcs.front().arc.state0_relative.velocity_mps),
              40.0);
    EXPECT_TRUE(std::any_of(result.planned_patch_events.begin(),
                            result.planned_patch_events.end(),
                            [](const Game::KeplerPatchEvent &event) {
                                return event.reason == Game::KeplerPatchBoundaryReason::Maneuver &&
                                       event.t_s == 0.0;
                            }));
}

TEST(KeplerPrediction, PatchedConicsDoesNotExtendPreviewPastEphemerisEnd)
{
    orbitsim::GameSimulation sim = make_static_transfer_sim();
    orbitsim::CelestialEphemeris ephemeris = make_static_ephemeris(sim, 0.0, 1'000.0);
    Game::KeplerPredictionBuildRequest request = make_short_planned_transfer_request(sim);
    request.ephemeris = &ephemeris;
    request.options.open_orbit_window_s = 30'000.0;
    const std::vector<Game::KeplerManeuverNode> nodes{
            Game::KeplerManeuverNode{
                    .node_id = 13,
                    .t_s = 60.0,
                    .primary_body_id = kEarthId,
                    .dv_rtn_mps = {0.0, 0.0, 0.0},
            },
    };
    request.maneuver_nodes = std::span<const Game::KeplerManeuverNode>(nodes.data(), nodes.size());

    const Game::KeplerPredictionBuildOutput result = Game::build_kepler_prediction(request);

    ASSERT_TRUE(result.valid) << Game::kepler_orbit_status_name(result.status);
    ASSERT_TRUE(result.planned_valid) << Game::kepler_orbit_status_name(result.planned_status);
    ASSERT_FALSE(result.planned_arcs.empty());
    EXPECT_GT(result.planned_arcs.back().arc.t1_s, request.requested_horizon_s);
    EXPECT_LE(result.planned_arcs.back().arc.t1_s, ephemeris.t_end_s());
}
