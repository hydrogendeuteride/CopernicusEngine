#pragma once

#include "game/orbit/kepler/kepler_types.h"

namespace Game
{
    [[nodiscard]] const char *kepler_orbit_status_name(KeplerOrbitStatus status);
    [[nodiscard]] const char *kepler_orbit_regime_name(orbitsim::KeplerOrbitRegime regime);
} // namespace Game
