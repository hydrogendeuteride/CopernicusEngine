#pragma once

#include "game/orbit/kepler/kepler_prediction_options.h"
#include "game/orbit/kepler/kepler_types.h"

#include "orbitsim/ephemeris.hpp"

namespace Game
{
    struct KeplerPrimaryResolveRequest
    {
        const orbitsim::GameSimulation *simulation{nullptr};
        const orbitsim::CelestialEphemeris *ephemeris{nullptr};
        orbitsim::Vec3 query_position_m{0.0, 0.0, 0.0};
        double query_time_s{0.0};
        orbitsim::BodyId fixed_primary_body_id{orbitsim::kInvalidBodyId};
        orbitsim::BodyId current_primary_body_id{orbitsim::kInvalidBodyId};
        orbitsim::SoiSwitchOptions switch_options{};
        double fallback_primary_hysteresis_keep_ratio{0.90};
    };

    [[nodiscard]] KeplerPrimaryResolution resolve_kepler_primary(const KeplerPrimaryResolveRequest &request);
} // namespace Game
