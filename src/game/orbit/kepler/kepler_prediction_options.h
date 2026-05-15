#pragma once

#include "orbitsim/kepler.hpp"
#include "orbitsim/soi.hpp"

#include <cstddef>

namespace Game
{
    struct KeplerCelestialNBodyEphemerisOptions
    {
        double min_dt_s{1.0};
        double max_dt_s{600.0};
        std::size_t soft_max_segments{3000};
        std::size_t hard_max_segments{3000};
        double pos_tolerance_m{10.0};
        double vel_tolerance_mps{1.0};
        double relative_tolerance_floor{1.0e-7};

        bool operator==(const KeplerCelestialNBodyEphemerisOptions &) const = default;
    };

    struct KeplerPatchedConicsOptions
    {
        bool enabled{true};
        std::size_t max_patches{8};
        double max_search_step_s{300.0};
        double refine_tolerance_s{0.25};
        double min_patch_duration_s{1.0e-3};

        bool operator==(const KeplerPatchedConicsOptions &) const = default;
    };

    struct KeplerPredictionOptions
    {
        double elliptic_period_count{1.0};
        double open_orbit_window_s{24.0 * 60.0 * 60.0};
        double fallback_primary_hysteresis_keep_ratio{0.90};
        double celestial_nbody_horizon_cap_s{24.0 * 60.0 * 60.0};
        double celestial_line_max_time_step_s{300.0};
        std::size_t celestial_line_max_vertices_per_track{256};
        orbitsim::SoiSwitchOptions primary_switch{};
        orbitsim::KeplerPropagationOptions propagation{};
        KeplerCelestialNBodyEphemerisOptions celestial_nbody_ephemeris{};
        KeplerPatchedConicsOptions patched_conics{};
    };

    struct KeplerArcLineOptions
    {
        double max_time_step_s{60.0};
        double min_time_step_s{1.0};
        double max_chord_error_m{1'000.0};
        std::size_t max_vertices_per_arc{8192};
        std::size_t max_vertices_total{16384};
        bool include_start{true};
        bool include_end{true};
        orbitsim::KeplerPropagationOptions propagation{};
    };
} // namespace Game
