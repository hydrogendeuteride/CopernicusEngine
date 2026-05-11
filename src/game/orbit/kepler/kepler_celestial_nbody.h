#pragma once

#include "game/orbit/kepler/kepler_prediction_options.h"
#include "game/orbit/kepler/kepler_types.h"

#include "orbitsim/ephemeris.hpp"
#include "orbitsim/game_sim.hpp"
#include "orbitsim/trajectory_segments.hpp"

#include <memory>

namespace Game
{
    using KeplerSharedCelestialEphemeris = std::shared_ptr<const orbitsim::CelestialEphemeris>;

    struct KeplerCelestialNBodyEphemerisRequest
    {
        const orbitsim::GameSimulation *simulation{nullptr};
        KeplerWorldFrame world_frame{};
        double t0_s{0.0};
        double requested_horizon_s{0.0};
        KeplerPredictionOptions options{};
    };

    struct KeplerCelestialNBodyEphemerisResult
    {
        bool valid{false};
        KeplerOrbitStatus status{KeplerOrbitStatus::InvalidInput};
        double horizon_s{0.0};
        KeplerSharedCelestialEphemeris ephemeris{};
        orbitsim::AdaptiveEphemerisDiagnostics diagnostics{};
        KeplerBodyStateProvider body_state_provider{};
    };

    struct KeplerCelestialNBodyLineRequest
    {
        KeplerSharedCelestialEphemeris ephemeris{};
        orbitsim::BodyId body_id{orbitsim::kInvalidBodyId};
        KeplerWorldFrame world_frame{};
        double t0_s{0.0};
        double requested_horizon_s{0.0};
        KeplerOrbitTessellationOptions tessellation{};
    };

    struct KeplerCelestialNBodyHorizonLimit
    {
        double uncapped_horizon_s{0.0};
        double horizon_s{0.0};
        double cap_s{0.0};
        bool capped{false};
    };

    KeplerCelestialNBodyHorizonLimit limit_kepler_celestial_nbody_horizon(
            double horizon_s,
            const KeplerPredictionOptions &options);

    double select_kepler_celestial_nbody_ephemeris_horizon_s(
            const KeplerCelestialNBodyEphemerisRequest &request);

    KeplerBodyStateProvider make_kepler_celestial_nbody_state_provider(
            KeplerSharedCelestialEphemeris ephemeris,
            const orbitsim::GameSimulation *fallback_simulation = nullptr);

    KeplerCelestialNBodyEphemerisResult build_kepler_celestial_nbody_ephemeris(
            const KeplerCelestialNBodyEphemerisRequest &request);

    KeplerOrbitLineSet build_kepler_celestial_nbody_lines(
            const KeplerCelestialNBodyLineRequest &request);
} // namespace Game
