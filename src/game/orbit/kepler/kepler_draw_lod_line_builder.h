#pragma once

#include "game/orbit/kepler/kepler_prediction_options.h"
#include "game/orbit/kepler/kepler_types.h"

#include "orbitsim/ephemeris.hpp"

#include <glm/mat4x4.hpp>

#include <span>

namespace Game
{
    struct KeplerDrawLodLineBuildRequest
    {
        std::span<const KeplerOrbitArc> arcs{};
        const orbitsim::CelestialEphemeris *ephemeris{nullptr};
        KeplerBodyStateProvider body_state_provider{};
        KeplerWorldFrame world_frame{};
        KeplerArcLineOptions line_options{};
        glm::mat4 viewproj{1.0f};
        WorldVec3 world_origin{0.0, 0.0, 0.0};
        double viewport_width_px{1280.0};
        double viewport_height_px{720.0};
        double render_error_px{0.75};
    };

    KeplerArcLineSet build_kepler_draw_lod_lines(const KeplerDrawLodLineBuildRequest &request);
} // namespace Game
