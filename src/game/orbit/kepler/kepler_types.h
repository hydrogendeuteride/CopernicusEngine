#pragma once

#include "core/world.h"

#include "orbitsim/game_sim.hpp"
#include "orbitsim/kepler.hpp"
#include "orbitsim/kepler_maneuver.hpp"
#include "orbitsim/kepler_trajectory.hpp"

#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

namespace Game
{
    enum class KeplerOrbitStatus : uint8_t
    {
        Ok = 0,
        InvalidInput,
        InvalidSimulation,
        InvalidSubjectState,
        PrimaryUnavailable,
        InvalidPrimary,
        InvalidArc,
        PrimaryMismatch,
        EphemerisUnavailable,
        ContinuityFailed,
        PropagationFailed,
        SampleBudgetExceeded,
        NoSamples,
    };

    enum class KeplerOrbitLineVertexFlags : uint32_t
    {
        None = 0u,
        OrbitStart = 1u << 0u,
        OrbitEnd = 1u << 1u,
        ArcStart = 1u << 2u,
        ArcEnd = 1u << 3u,
    };

    struct KeplerPrimaryResolution
    {
        bool valid{false};
        KeplerOrbitStatus status{KeplerOrbitStatus::PrimaryUnavailable};
        orbitsim::BodyId body_id{orbitsim::kInvalidBodyId};
        double mass_kg{0.0};
        double radius_m{0.0};
        double mu_m3_s2{0.0};
        orbitsim::State state_inertial{};
    };

    struct KeplerOrbitArc
    {
        orbitsim::KeplerArc arc{};
        orbitsim::State primary_state_inertial_at_t0{};
    };

    struct KeplerOrbitBuildResult
    {
        bool valid{false};
        KeplerOrbitStatus status{KeplerOrbitStatus::InvalidInput};
        KeplerPrimaryResolution primary{};
        KeplerOrbitArc base_arc{};
        double horizon_s{0.0};
    };

    struct KeplerManeuverNode
    {
        int node_id{-1};
        double t_s{std::numeric_limits<double>::quiet_NaN()};
        orbitsim::BodyId primary_body_id{orbitsim::kInvalidBodyId};
        orbitsim::Vec3 dv_rtn_mps{0.0, 0.0, 0.0};
    };

    struct KeplerManeuverSolveResult
    {
        bool valid{false};
        KeplerOrbitStatus status{KeplerOrbitStatus::InvalidInput};
        std::vector<KeplerOrbitArc> arcs{};
        orbitsim::KeplerManeuverDiagnostics diagnostics{};
    };

    struct KeplerBodyStateProvider
    {
        std::function<bool(orbitsim::BodyId body_id, double t_s, orbitsim::State &out_state)> state_at{};
    };

    struct KeplerWorldFrame
    {
        WorldVec3 world_reference_body_world{0.0, 0.0, 0.0};
        orbitsim::BodyId world_reference_body_id{orbitsim::kInvalidBodyId};
        orbitsim::State world_reference_state_inertial{};
    };

    struct KeplerOrbitLineVertex
    {
        WorldVec3 position_world{0.0, 0.0, 0.0};
        double t_s{std::numeric_limits<double>::quiet_NaN()};
        orbitsim::BodyId primary_body_id{orbitsim::kInvalidBodyId};
        uint32_t flags{0u};
    };

    struct KeplerOrbitTessellationDiagnostics
    {
        KeplerOrbitStatus status{KeplerOrbitStatus::InvalidInput};
        std::size_t requested_arcs{0};
        std::size_t sampled_arcs{0};
        std::size_t requested_samples{0};
        std::size_t accepted_samples{0};
        std::size_t failed_arc_index{0};
        orbitsim::KeplerStatus first_kepler_failure{orbitsim::KeplerStatus::Ok};
        bool budget_hit{false};
    };

    struct KeplerOrbitLineSet
    {
        bool valid{false};
        std::vector<KeplerOrbitLineVertex> vertices{};
        KeplerOrbitTessellationDiagnostics diagnostics{};
    };
} // namespace Game
