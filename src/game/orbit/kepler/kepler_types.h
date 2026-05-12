#pragma once

#include "core/world.h"

#include "orbitsim/game_sim.hpp"
#include "orbitsim/kepler.hpp"
#include "orbitsim/kepler_maneuver.hpp"
#include "orbitsim/kepler_trajectory.hpp"

#include <cmath>
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

    enum class KeplerArcLineVertexFlags : uint32_t
    {
        None = 0u,
        OrbitStart = 1u << 0u,
        OrbitEnd = 1u << 1u,
        ArcStart = 1u << 2u,
        ArcEnd = 1u << 3u,
    };

    // Selected primary body and its state.
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

    // Kepler arc, primary state at t0.
    struct KeplerOrbitArc
    {
        orbitsim::KeplerArc arc{};
        orbitsim::State primary_state_inertial_at_t0{};
    };

    // Base arc build result for one subject.
    struct KeplerArcBuildResult
    {
        bool valid{false};
        KeplerOrbitStatus status{KeplerOrbitStatus::InvalidInput};
        KeplerPrimaryResolution primary{};
        KeplerOrbitArc base_arc{};
        double horizon_s{0.0};
    };

    // Maneuver node as a timed RTN impulse.
    struct KeplerManeuverNode
    {
        int node_id{-1};
        double t_s{std::numeric_limits<double>::quiet_NaN()};
        orbitsim::BodyId primary_body_id{orbitsim::kInvalidBodyId};
        orbitsim::Vec3 dv_rtn_mps{0.0, 0.0, 0.0};
    };

    // Maneuvered arc chain and diagnostics.
    struct KeplerManeuverArcBuildResult
    {
        bool valid{false};
        KeplerOrbitStatus status{KeplerOrbitStatus::InvalidInput};
        std::vector<KeplerOrbitArc> arcs{};
        orbitsim::KeplerManeuverDiagnostics diagnostics{};
    };

    // Moving-body state sampler.
    struct KeplerBodyStateProvider
    {
        std::function<bool(orbitsim::BodyId body_id, double t_s, orbitsim::State &out_state)> state_at{};
    };

    // Inertial-to-world conversion frame.
    struct KeplerWorldFrame
    {
        WorldVec3 world_reference_body_world{0.0, 0.0, 0.0};
        orbitsim::BodyId world_reference_body_id{orbitsim::kInvalidBodyId};
        orbitsim::State world_reference_state_inertial{};
    };

    // Sampled orbit point in world space.
    struct KeplerArcLineVertex
    {
        WorldVec3 position_world{0.0, 0.0, 0.0};
        double t_s{std::numeric_limits<double>::quiet_NaN()};
        orbitsim::BodyId primary_body_id{orbitsim::kInvalidBodyId};
        uint32_t flags{0u};
    };

    // Orbit line sampling diagnostics.
    struct KeplerArcLineDiagnostics
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

    // Polyline output for one or more arcs.
    struct KeplerArcLineSet
    {
        bool valid{false};
        std::vector<KeplerArcLineVertex> vertices{};
        KeplerArcLineDiagnostics diagnostics{};
    };

    inline bool kepler_finite_vec3(const orbitsim::Vec3 &v)
    {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }

    inline double kepler_positive_or_default(const double value,
                                             const double fallback)
    {
        return (std::isfinite(value) && value > 0.0) ? value : fallback;
    }

    inline bool kepler_same_sample_time(const double a,
                                        const double b)
    {
        return std::isfinite(a) && std::isfinite(b) && std::abs(a - b) <= 1.0e-9;
    }
} // namespace Game
