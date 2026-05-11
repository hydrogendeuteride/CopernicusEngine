#pragma once

#include "game/orbit/kepler/kepler_prediction_options.h"
#include "game/orbit/kepler/kepler_primary_resolver.h"
#include "game/orbit/kepler/kepler_types.h"

namespace Game
{
    struct KeplerOrbitBuildRequest
    {
        const orbitsim::GameSimulation *simulation{nullptr};
        const orbitsim::CelestialEphemeris *ephemeris{nullptr};
        orbitsim::State subject_state_inertial{};
        double t0_s{0.0};
        double requested_horizon_s{0.0};
        orbitsim::BodyId fixed_primary_body_id{orbitsim::kInvalidBodyId};
        orbitsim::BodyId current_primary_body_id{orbitsim::kInvalidBodyId};
        KeplerPredictionOptions options{};
    };

    double select_kepler_horizon_s(const orbitsim::KeplerArc &arc,
                                   const KeplerPredictionOptions &options);

    KeplerOrbitBuildResult build_kepler_orbit(const KeplerOrbitBuildRequest &request);
} // namespace Game
