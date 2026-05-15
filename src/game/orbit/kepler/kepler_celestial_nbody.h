#pragma once

#include "game/orbit/kepler/kepler_prediction_options.h"
#include "game/orbit/kepler/kepler_types.h"

#include "orbitsim/ephemeris.hpp"
#include "orbitsim/game_sim.hpp"
#include "orbitsim/trajectory_segments.hpp"

#include <memory>

namespace Game
{
    // Cached ephemeris shared by tracks and body-state providers.
    using KeplerSharedCelestialEphemeris = std::shared_ptr<const orbitsim::CelestialEphemeris>;

    // N-body ephemeris build request; integration stays in inertial space.
    struct KeplerCelestialNBodyEphemerisRequest
    {
        const orbitsim::GameSimulation *simulation{nullptr};
        KeplerWorldFrame world_frame{};
        double t0_s{0.0};
        double requested_horizon_s{0.0};
        KeplerPredictionOptions options{};
    };

    // Ephemeris plus lookup provider for moving primary/reference bodies.
    struct KeplerCelestialNBodyEphemerisResult
    {
        bool valid{false};
        KeplerOrbitStatus status{KeplerOrbitStatus::InvalidInput};
        double horizon_s{0.0};
        KeplerSharedCelestialEphemeris ephemeris{};
        orbitsim::AdaptiveEphemerisDiagnostics diagnostics{};
        KeplerBodyStateProvider body_state_provider{};
    };

    // Samples one ephemeris body into world-space OrbitPlot vertices.
    struct KeplerCelestialNBodyLineRequest
    {
        KeplerSharedCelestialEphemeris ephemeris{};
        orbitsim::BodyId body_id{orbitsim::kInvalidBodyId};
        KeplerWorldFrame world_frame{};
        double t0_s{0.0};
        double requested_horizon_s{0.0};
        KeplerArcLineOptions line_options{};
    };

    // Horizon after n-body cap, with original value kept for diagnostics.
    struct KeplerCelestialNBodyHorizonLimit
    {
        double uncapped_horizon_s{0.0};
        double horizon_s{0.0};
        double cap_s{0.0};
        bool capped{false};
    };

    // Apply KeplerPredictionOptions::celestial_nbody_horizon_cap_s.
    KeplerCelestialNBodyHorizonLimit limit_kepler_celestial_nbody_horizon(
            double horizon_s,
            const KeplerPredictionOptions &options);

    // Apply the configured cap without cutting below a correctness-critical floor.
    KeplerCelestialNBodyHorizonLimit limit_kepler_celestial_nbody_horizon(
            double horizon_s,
            const KeplerPredictionOptions &options,
            double horizon_floor_s);

    // Integrate massive bodies into an adaptive ephemeris.
    KeplerCelestialNBodyEphemerisResult build_kepler_celestial_nbody_ephemeris(
            const KeplerCelestialNBodyEphemerisRequest &request);

    // Build a world-space line strip for one ephemeris body.
    KeplerArcLineSet build_kepler_celestial_nbody_lines(
            const KeplerCelestialNBodyLineRequest &request);
} // namespace Game
