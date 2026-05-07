#pragma once

#include "game/orbit/kepler/kepler_prediction_options.h"
#include "game/orbit/kepler/kepler_types.h"

#include <span>

namespace Game
{
    struct KeplerOrbitTessellationRequest
    {
        std::span<const KeplerOrbitArc> arcs{};
        KeplerOrbitTessellationOptions options{};
        KeplerBodyStateProvider body_state_provider{};
        KeplerWorldFrame world_frame{};
    };

    [[nodiscard]] KeplerOrbitLineSet build_kepler_orbit_lines(const KeplerOrbitTessellationRequest &request);
} // namespace Game
