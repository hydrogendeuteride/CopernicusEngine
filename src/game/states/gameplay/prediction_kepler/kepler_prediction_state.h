#pragma once

#include "game/orbit/kepler/kepler_arc_info.h"
#include "game/orbit/kepler/kepler_types.h"
#include "game/entity.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace Game
{
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
            std::vector<KeplerOrbitArc> base_arcs{};
            std::vector<KeplerOrbitArc> planned_arcs{};
            KeplerArcLineSet base_lines{};
            KeplerArcLineSet planned_lines{};
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
            tracks.clear();
            celestial_nbody_ephemeris = {};
        }
    };
} // namespace Game
