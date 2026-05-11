#pragma once

#include "game/orbit/kepler/kepler_types.h"

#include <span>

namespace Game
{
    KeplerManeuverSolveResult build_kepler_maneuver_arcs(
            const KeplerOrbitArc &base_arc,
            std::span<const KeplerManeuverNode> nodes,
            const orbitsim::KeplerPropagationOptions &propagation_options = {});
} // namespace Game
