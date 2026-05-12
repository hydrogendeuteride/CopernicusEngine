#pragma once

#include "game/orbit/kepler/kepler_prediction_options.h"
#include "game/orbit/kepler/kepler_primary_resolver.h"
#include "game/orbit/kepler/kepler_types.h"

#include <span>

namespace Game
{
    // Inputs for building a primary-relative arc.
    struct KeplerArcBuildRequest
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

    // Selects the default prediction span.
    double select_kepler_arc_horizon_s(const orbitsim::KeplerArc &arc,
                                       const KeplerPredictionOptions &options);

    // Builds the subject's base Kepler arc.
    KeplerArcBuildResult build_kepler_arc(const KeplerArcBuildRequest &request);

    // Applies maneuver nodes and returns arc segments.
    KeplerManeuverArcBuildResult build_kepler_maneuver_arc_chain(
            const KeplerOrbitArc &base_arc,
            std::span<const KeplerManeuverNode> nodes,
            const orbitsim::KeplerPropagationOptions &propagation_options = {});
} // namespace Game
