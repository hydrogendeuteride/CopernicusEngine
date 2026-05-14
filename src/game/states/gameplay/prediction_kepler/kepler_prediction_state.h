#pragma once

#include "game/orbit/kepler/kepler_arc_info.h"
#include "game/orbit/kepler/kepler_types.h"
#include "game/entity.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace Game
{
    struct KeplerPropagationOptionsFingerprint
    {
        int max_iterations{0};
        double abs_tolerance{0.0};
        double rel_tolerance{0.0};
        double step_tolerance{0.0};
        double rel_step_tolerance{0.0};
        double near_parabolic_alpha_tolerance{0.0};
        double max_hyperbolic_stumpff_arg{0.0};

        bool operator==(const KeplerPropagationOptionsFingerprint &) const = default;
    };

    struct KeplerSoiSwitchOptionsFingerprint
    {
        double enter_scale{0.0};
        double exit_scale{0.0};
        bool prefer_smallest_soi{false};
        bool fallback_to_max_accel{false};

        bool operator==(const KeplerSoiSwitchOptionsFingerprint &) const = default;
    };

    struct KeplerCelestialNBodyEphemerisFingerprint
    {
        double min_dt_s{0.0};
        double max_dt_s{0.0};
        std::size_t soft_max_segments{0};
        std::size_t hard_max_segments{0};
        double pos_tolerance_m{0.0};
        double vel_tolerance_mps{0.0};
        double relative_tolerance_floor{0.0};

        bool operator==(const KeplerCelestialNBodyEphemerisFingerprint &) const = default;
    };

    struct KeplerPredictionInputFingerprint
    {
        double requested_horizon_s{0.0};
        orbitsim::BodyId fixed_primary_body_id{orbitsim::kInvalidBodyId};
        orbitsim::BodyId world_reference_body_id{orbitsim::kInvalidBodyId};
        uint64_t maneuver_revision{0};
        bool build_celestial_kepler_tracks{false};
        bool build_celestial_nbody_tracks{false};

        double elliptic_period_count{0.0};
        double open_orbit_window_s{0.0};
        double fallback_primary_hysteresis_keep_ratio{0.0};
        double celestial_nbody_horizon_cap_s{0.0};
        double celestial_line_max_time_step_s{0.0};
        std::size_t celestial_line_max_vertices_per_track{0};
        KeplerSoiSwitchOptionsFingerprint primary_switch{};
        KeplerPropagationOptionsFingerprint prediction_propagation{};
        KeplerCelestialNBodyEphemerisFingerprint celestial_nbody_ephemeris{};

        double line_max_time_step_s{0.0};
        double line_min_time_step_s{0.0};
        double line_max_chord_error_m{0.0};
        std::size_t line_max_vertices_per_arc{0};
        std::size_t line_max_vertices_total{0};
        bool line_include_start{false};
        bool line_include_end{false};
        KeplerPropagationOptionsFingerprint line_propagation{};

        bool operator==(const KeplerPredictionInputFingerprint &) const = default;
    };

    // Runtime state for the gameplay Kepler prediction system.
    struct KeplerPredictionState
    {
        // One drawable/pickable predicted orbit track.
        struct Track
        {
            bool valid{false};
            bool celestial{false};
            bool celestial_nbody{false};
            bool active_player{false};
            EntityId entity{};
            orbitsim::BodyId body_id{orbitsim::kInvalidBodyId};
            std::string label{};
            glm::vec3 orbit_rgb{0.18f, 0.82f, 1.0f};

            KeplerOrbitStatus status{KeplerOrbitStatus::InvalidInput};
            double horizon_s{0.0};
            orbitsim::BodyId primary_body_id{orbitsim::kInvalidBodyId};

            KeplerArcBuildResult orbit{};
            KeplerWorldFrame world_frame{};
            KeplerBodyStateProvider body_state_provider{};
            orbitsim::KeplerPropagationOptions line_propagation{};
            std::vector<KeplerOrbitArc> base_arcs{};
            std::vector<KeplerOrbitArc> planned_arcs{};
            KeplerArcLineSet base_lines{};
            KeplerArcLineSet planned_lines{};
            bool planned_requested{false};
            bool planned_valid{false};
            KeplerOrbitStatus planned_status{KeplerOrbitStatus::Ok};
            orbitsim::KeplerManeuverDiagnostics planned_diagnostics{};
            KeplerArcLineDiagnostics planned_line_diagnostics{};
            KeplerArcMetrics metrics{};
        };

        // Debug snapshot for the shared celestial n-body ephemeris cache.
        struct CelestialNBodyEphemerisDebug
        {
            bool valid{false};
            KeplerOrbitStatus status{KeplerOrbitStatus::InvalidInput};
            double uncapped_required_horizon_s{0.0};
            double required_horizon_s{0.0};
            double built_horizon_s{0.0};
            double horizon_cap_s{0.0};
            double t0_s{0.0};
            double t_end_s{0.0};
            std::size_t body_count{0};
            std::size_t accepted_segments{0};
            std::size_t rejected_splits{0};
            std::size_t forced_boundary_splits{0};
            double min_dt_s{0.0};
            double avg_dt_s{0.0};
            double max_dt_s{0.0};
            bool horizon_capped{false};
            bool hard_cap_hit{false};
            bool cancelled{false};
        };

        bool enabled{true};
        bool valid{false};
        bool dirty{true};

        uint64_t revision{0};
        uint64_t maneuver_revision{0};
        KeplerPredictionInputFingerprint input_fingerprint{};

        KeplerOrbitStatus status{KeplerOrbitStatus::InvalidInput};
        double build_time_s{0.0};
        double build_wall_time_s{0.0};
        double horizon_s{0.0};

        orbitsim::BodyId primary_body_id{orbitsim::kInvalidBodyId};
        orbitsim::BodyId world_reference_body_id{orbitsim::kInvalidBodyId};

        std::vector<Track> tracks{};
        CelestialNBodyEphemerisDebug celestial_nbody_ephemeris{};

        void clear_result(const KeplerOrbitStatus new_status = KeplerOrbitStatus::InvalidInput)
        {
            valid = false;
            status = new_status;
            build_wall_time_s = 0.0;
            horizon_s = 0.0;
            primary_body_id = orbitsim::kInvalidBodyId;
            world_reference_body_id = orbitsim::kInvalidBodyId;
            input_fingerprint = {};
            tracks.clear();
            celestial_nbody_ephemeris = {};
        }
    };
} // namespace Game
