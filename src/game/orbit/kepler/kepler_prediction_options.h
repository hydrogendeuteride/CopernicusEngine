#pragma once

#include "orbitsim/kepler.hpp"
#include "orbitsim/soi.hpp"

#include <cstddef>

namespace Game
{
    struct KeplerPredictionOptions
    {
        double elliptic_period_count{1.0};
        double open_orbit_window_s{30.0 * 24.0 * 60.0 * 60.0};
        double fallback_primary_hysteresis_keep_ratio{0.90};
        orbitsim::SoiSwitchOptions primary_switch{};
        orbitsim::KeplerPropagationOptions propagation{};
    };

    struct KeplerOrbitTessellationOptions
    {
        double max_time_step_s{60.0};
        double min_time_step_s{1.0};
        std::size_t max_vertices_per_arc{2048};
        std::size_t max_vertices_total{8192};
        bool include_start{true};
        bool include_end{true};
        orbitsim::KeplerPropagationOptions propagation{};
    };
} // namespace Game
