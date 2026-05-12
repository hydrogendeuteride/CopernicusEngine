#pragma once

#include "game/orbit/kepler/kepler_prediction_options.h"
#include "game/orbit/kepler/kepler_types.h"

#include <span>

namespace Game
{
    // Inputs for sampling arcs into world-space line vertices.
    struct KeplerArcLineBuildRequest
    {
        std::span<const KeplerOrbitArc> arcs{};
        KeplerArcLineOptions options{};
        KeplerBodyStateProvider body_state_provider{};
        KeplerWorldFrame world_frame{};
    };

    // Samples one or more arcs into a drawable orbit polyline.
    KeplerArcLineSet build_kepler_arc_lines(const KeplerArcLineBuildRequest &request);
} // namespace Game
