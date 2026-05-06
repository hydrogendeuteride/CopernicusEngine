#pragma once

#include "orbitsim/game_sim.hpp"
#include "orbitsim/trajectory_segments.hpp"
#include "orbitsim/trajectory_types.hpp"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace Game
{
    using OrbitPredictionSharedCelestialEphemeris = std::shared_ptr<const orbitsim::CelestialEphemeris>;

    enum class OrbitPredictionRequestKind : uint8_t
    {
        Spacecraft = 0,
        Celestial,
    };

    enum class OrbitPredictionSolveQuality : uint8_t
    {
        Full = 0,
        FastPreview = 1,
    };

    enum class OrbitPredictionRequestPriority : uint8_t
    {
        BackgroundCelestial = 0,
        BackgroundOrbiter,
        Overlay,
        ActiveTrack,
        ActiveInteractiveTrack,
    };

    enum class OrbitPredictionStatus : uint8_t
    {
        None = 0,
        Success,
        InvalidInput,
        InvalidSubject,
        InvalidSamplingSpec,
        EphemerisUnavailable,
        TrajectorySegmentsUnavailable,
        TrajectorySamplesUnavailable,
        ContinuityFailed,
        Cancelled,
    };

    struct OrbitPredictionAdaptiveStageDiagnostics
    {
        double requested_duration_s{0.0};
        double covered_duration_s{0.0};
        std::size_t accepted_segments{0};
        std::size_t rejected_splits{0};
        std::size_t forced_boundary_splits{0};
        std::size_t frame_resegmentation_count{0};
        double min_dt_s{0.0};
        double max_dt_s{0.0};
        double avg_dt_s{0.0};
        bool hard_cap_hit{false};
        bool cancelled{false};
        bool cache_reused{false};
        std::size_t maneuver_apply_failed_count{0};
        int maneuver_apply_failed_node_id{-1};
    };

    struct OrbitPredictionDiagnostics
    {
        OrbitPredictionStatus status{OrbitPredictionStatus::None};
        bool cancelled{false};
        std::size_t ephemeris_segment_count{0};
        std::size_t trajectory_segment_count{0};
        std::size_t trajectory_segment_count_planned{0};
        std::size_t trajectory_sample_count{0};
        std::size_t trajectory_sample_count_planned{0};
        OrbitPredictionAdaptiveStageDiagnostics ephemeris{};
        OrbitPredictionAdaptiveStageDiagnostics trajectory_base{};
        OrbitPredictionAdaptiveStageDiagnostics trajectory_planned{};
    };

    struct OrbitPredictionManeuverNodePreview
    {
        int node_id{-1};
        double t_s{0.0};
        bool valid{false};
        orbitsim::Vec3 inertial_position_m{0.0, 0.0, 0.0};
        orbitsim::Vec3 inertial_velocity_mps{0.0, 0.0, 0.0};
    };

    struct OrbitPredictionManeuverImpulse
    {
        int node_id{-1};
        double t_s{0.0};
        orbitsim::BodyId primary_body_id{orbitsim::kInvalidBodyId};
        // Gameplay UI authors node DV directly in true RTN using the pre-burn primary-relative state.
        orbitsim::Vec3 dv_rtn_mps{0.0, 0.0, 0.0};
    };

    enum class OrbitPredictionProfileId : uint8_t
    {
        Exact = 0,
        Near,
        Tail,
    };

    enum class OrbitPredictionChunkBoundaryFlags : uint32_t
    {
        None = 0u,
        RequestStart = 1u << 0u,
        RequestEnd = 1u << 1u,
        Maneuver = 1u << 2u,
        KnownDiscontinuity = 1u << 3u,
        TimeBand = 1u << 4u,
        PreviewAnchor = 1u << 5u,
        PreviewChunk = 1u << 6u,
    };

    enum class OrbitPredictionChunkQualityState : uint8_t
    {
        Final = 0,
        PreviewPatch,
    };

    enum class OrbitPredictionPublishStage : uint8_t
    {
        Final = 0,
        PreviewFinalizing = Final,
        PreviewStreaming,
        FullStreaming,
    };

    struct OrbitPredictionProfileDefinition
    {
        OrbitPredictionProfileId profile_id{OrbitPredictionProfileId::Near};
        double integrator_tolerance_multiplier{1.0};
        double min_dt_s{0.0};
        double max_dt_s{0.0};
        double lookup_max_dt_s{0.0};
        std::size_t soft_max_segments{0};
        double ephemeris_min_dt_s{0.0};
        double ephemeris_max_dt_s{0.0};
        std::size_t ephemeris_soft_max_segments{0};
        double output_sample_density_scale{1.0};
        double seam_overlap_s{0.0};
    };

    struct OrbitPredictionChunkPlan
    {
        uint32_t chunk_id{0};
        double t0_s{std::numeric_limits<double>::quiet_NaN()};
        double t1_s{std::numeric_limits<double>::quiet_NaN()};
        OrbitPredictionProfileId profile_id{OrbitPredictionProfileId::Near};
        uint32_t boundary_flags{0u};
        uint32_t priority{0u};
        bool allow_reuse{true};
        bool requires_seam_validation{false};
    };

    struct OrbitPredictionPublishedChunk
    {
        uint32_t chunk_id{0};
        OrbitPredictionChunkQualityState quality_state{OrbitPredictionChunkQualityState::Final};
        double t0_s{std::numeric_limits<double>::quiet_NaN()};
        double t1_s{std::numeric_limits<double>::quiet_NaN()};
        bool includes_planned_path{false};
        bool reused_from_cache{false};
    };

    struct OrbitPredictionStreamedPlannedChunk
    {
        OrbitPredictionPublishedChunk published_chunk{};
        std::vector<orbitsim::TrajectorySegment> trajectory_segments_inertial{};
        std::vector<orbitsim::TrajectorySample> trajectory_inertial{};
        std::vector<OrbitPredictionManeuverNodePreview> maneuver_previews{};
        OrbitPredictionAdaptiveStageDiagnostics diagnostics{};
        orbitsim::State start_state{};
        orbitsim::State end_state{};
    };

    struct OrbitPredictionSolvePlan
    {
        bool valid{false};
        double t0_s{std::numeric_limits<double>::quiet_NaN()};
        double t1_s{std::numeric_limits<double>::quiet_NaN()};
        std::vector<OrbitPredictionChunkPlan> chunks{};
    };

    struct OrbitPredictionChunkActivityProbe
    {
        bool valid{false};
        double heading_change_rad{0.0};
        double accel_magnitude_mps2{0.0};
        double jerk_magnitude_mps3{0.0};
        double dominant_gravity_ratio{1.0};
        double maneuver_proximity_s{std::numeric_limits<double>::infinity()};
        orbitsim::BodyId primary_body_id_start{orbitsim::kInvalidBodyId};
        orbitsim::BodyId primary_body_id_mid{orbitsim::kInvalidBodyId};
        orbitsim::BodyId primary_body_id_end{orbitsim::kInvalidBodyId};
        OrbitPredictionProfileId recommended_profile_id{OrbitPredictionProfileId::Near};
        bool should_split{false};
    };

    struct OrbitPredictionChunkSeamDiagnostics
    {
        bool valid{false};
        bool success{false};
        double sample_time_s{std::numeric_limits<double>::quiet_NaN()};
        double time_error_s{0.0};
        double position_error_m{0.0};
        double velocity_error_mps{0.0};
        orbitsim::BodyId previous_primary_body_id{orbitsim::kInvalidBodyId};
        orbitsim::BodyId current_primary_body_id{orbitsim::kInvalidBodyId};
        bool primary_flutter{false};
        uint32_t retry_count{0};
    };

    struct OrbitPredictionRequest
    {
        struct Envelope
        {
            // The worker handles both spacecraft and celestial prediction jobs.
            OrbitPredictionRequestKind kind{OrbitPredictionRequestKind::Spacecraft};
            uint64_t track_id{0};
            uint64_t maneuver_plan_revision{0};
            bool maneuver_plan_signature_valid{false};
            uint64_t maneuver_plan_signature{0};
            OrbitPredictionRequestPriority priority{OrbitPredictionRequestPriority::BackgroundOrbiter};
        };

        struct World
        {
            double sim_time_s{0.0};
            orbitsim::GameSimulation::Config sim_config{};
            std::vector<orbitsim::MassiveBody> massive_bodies;
            OrbitPredictionSharedCelestialEphemeris shared_ephemeris{};
        };

        struct Subject
        {
            // Celestial jobs identify the predicted body directly.
            orbitsim::BodyId subject_body_id{orbitsim::kInvalidBodyId};
            orbitsim::Vec3 ship_bary_position_m{0.0, 0.0, 0.0};
            orbitsim::Vec3 ship_bary_velocity_mps{0.0, 0.0, 0.0};
            orbitsim::BodyId preferred_primary_body_id{orbitsim::kInvalidBodyId};
        };

        struct Options
        {
            bool thrusting{false};
            bool lagrange_sensitive{false};
            OrbitPredictionSolveQuality solve_quality{OrbitPredictionSolveQuality::Full};
            double future_window_s{600.0};
            double celestial_ephemeris_dt_s{0.0};
        };

        struct PreviewPatchSpec
        {
            bool active{false};
            bool anchor_state_valid{false};
            bool anchor_state_trusted{false};
            double anchor_time_s{std::numeric_limits<double>::quiet_NaN()};
            double visual_window_s{0.0};
            double exact_window_s{0.0};
            orbitsim::State anchor_state_inertial{};
        };

        struct PlannedSuffixRefineSpec
        {
            bool active{false};
            int anchor_node_id{-1};
            double anchor_time_s{std::numeric_limits<double>::quiet_NaN()};
            orbitsim::State anchor_state_inertial{};
            std::vector<orbitsim::TrajectorySegment> prefix_segments_inertial{};
            std::vector<OrbitPredictionManeuverNodePreview> prefix_previews{};
        };

        struct FullStreamPublishSpec
        {
            bool active{false};
            double min_publish_interval_s{0.0};
        };

        struct ManeuverInput
        {
            std::vector<OrbitPredictionManeuverImpulse> maneuver_impulses;
            PreviewPatchSpec preview_patch{};
            PlannedSuffixRefineSpec planned_suffix_refine{};
            FullStreamPublishSpec full_stream_publish{};
        };

        Envelope envelope{};
        World world{};
        Subject subject{};
        Options options{};
        ManeuverInput maneuver{};
    };

    struct OrbitPredictionResult
    {
        struct CoreData
        {
            OrbitPredictionSharedCelestialEphemeris shared_ephemeris{};
            std::vector<orbitsim::MassiveBody> massive_bodies{};
            std::vector<orbitsim::TrajectorySample> trajectory_inertial{};
            std::vector<orbitsim::TrajectorySegment> trajectory_segments_inertial{};
        };
        using SharedCoreData = std::shared_ptr<const CoreData>;

        struct Envelope
        {
            uint64_t track_id{0};
            uint64_t generation_id{0};
            uint64_t maneuver_plan_revision{0};
            bool maneuver_plan_signature_valid{false};
            uint64_t maneuver_plan_signature{0};
            bool valid{false};
            bool baseline_reused{false};
            OrbitPredictionSolveQuality solve_quality{OrbitPredictionSolveQuality::Full};
            OrbitPredictionPublishStage publish_stage{OrbitPredictionPublishStage::Final};
        };

        struct Timing
        {
            double build_time_s{0.0};
            double compute_time_ms{0.0};
        };

        struct CorePayload
        {
            OrbitPredictionSharedCelestialEphemeris shared_ephemeris{};
            std::vector<orbitsim::MassiveBody> massive_bodies{};
            std::vector<orbitsim::TrajectorySample> trajectory_inertial{};
            std::vector<orbitsim::TrajectorySegment> trajectory_segments_inertial{};
            SharedCoreData shared_core_data{};
        };

        struct PlannedPayload
        {
            std::vector<orbitsim::TrajectorySample> trajectory_inertial{};
            std::vector<orbitsim::TrajectorySegment> trajectory_segments_inertial{};
            std::vector<OrbitPredictionManeuverNodePreview> maneuver_previews{};
        };

        struct PublishPayload
        {
            std::vector<OrbitPredictionPublishedChunk> published_chunks{};
            std::vector<OrbitPredictionStreamedPlannedChunk> streamed_planned_chunks{};
        };

        Envelope envelope{};
        Timing timing{};
        CorePayload core{};
        PlannedPayload planned{};
        PublishPayload publish{};
        OrbitPredictionDiagnostics diagnostics{};

        [[nodiscard]] const OrbitPredictionSharedCelestialEphemeris &resolved_shared_ephemeris() const
        {
            return core.shared_core_data ? core.shared_core_data->shared_ephemeris : core.shared_ephemeris;
        }

        [[nodiscard]] const std::vector<orbitsim::MassiveBody> &resolved_massive_bodies() const
        {
            return core.shared_core_data ? core.shared_core_data->massive_bodies : core.massive_bodies;
        }

        [[nodiscard]] const std::vector<orbitsim::TrajectorySample> &resolved_trajectory_inertial() const
        {
            return core.shared_core_data ? core.shared_core_data->trajectory_inertial : core.trajectory_inertial;
        }

        [[nodiscard]] const std::vector<orbitsim::TrajectorySegment> &resolved_trajectory_segments_inertial() const
        {
            return core.shared_core_data ? core.shared_core_data->trajectory_segments_inertial
                                         : core.trajectory_segments_inertial;
        }

        [[nodiscard]] const SharedCoreData &shared_core_data() const
        {
            return core.shared_core_data;
        }

        [[nodiscard]] bool has_shared_core_data() const
        {
            return static_cast<bool>(core.shared_core_data);
        }

        [[nodiscard]] std::vector<orbitsim::MassiveBody> clone_massive_bodies() const
        {
            return resolved_massive_bodies();
        }

        [[nodiscard]] std::vector<orbitsim::TrajectorySample> clone_trajectory_inertial() const
        {
            return resolved_trajectory_inertial();
        }

        [[nodiscard]] std::vector<orbitsim::TrajectorySegment> clone_trajectory_segments_inertial() const
        {
            return resolved_trajectory_segments_inertial();
        }

        [[nodiscard]] std::vector<orbitsim::MassiveBody> take_massive_bodies()
        {
            assert(!core.shared_core_data &&
                   "shared core data cannot be taken; use resolved_*(), shared_core_data(), or clone_*()");
            return std::move(core.massive_bodies);
        }

        [[nodiscard]] std::vector<orbitsim::TrajectorySample> take_trajectory_inertial()
        {
            assert(!core.shared_core_data &&
                   "shared core data cannot be taken; use resolved_*(), shared_core_data(), or clone_*()");
            return std::move(core.trajectory_inertial);
        }

        [[nodiscard]] std::vector<orbitsim::TrajectorySegment> take_trajectory_segments_inertial()
        {
            assert(!core.shared_core_data &&
                   "shared core data cannot be taken; use resolved_*(), shared_core_data(), or clone_*()");
            return std::move(core.trajectory_segments_inertial);
        }

        void set_shared_core_data(SharedCoreData shared_core_data)
        {
            core.shared_core_data = std::move(shared_core_data);
        }
    };

    struct OrbitPredictionEphemerisSamplingSpec
    {
        bool valid{false};
        double horizon_s{0.0};
    };

    struct OrbitPredictionEphemerisBuildRequest
    {
        double sim_time_s{0.0};
        orbitsim::GameSimulation::Config sim_config{};
        std::vector<orbitsim::MassiveBody> massive_bodies;
        double duration_s{0.0};
        orbitsim::AdaptiveEphemerisOptions adaptive_options{};
    };

    struct OrbitPredictionCachedEphemerisEntry
    {
        double sim_time_s{0.0};
        double duration_s{0.0};
        orbitsim::GameSimulation::Config sim_config{};
        std::vector<orbitsim::MassiveBody> massive_bodies{};
        orbitsim::AdaptiveEphemerisOptions adaptive_options{};
        OrbitPredictionSharedCelestialEphemeris ephemeris{};
        OrbitPredictionAdaptiveStageDiagnostics diagnostics{};
        uint64_t last_use_serial{0};
    };

    struct OrbitPredictionPlannedChunkCacheKey
    {
        uint64_t track_id{0};
        uint64_t baseline_generation_id{0};
        uint64_t upstream_maneuver_hash{0};
        uint64_t frame_independent_generation{0};
        double chunk_t0_s{std::numeric_limits<double>::quiet_NaN()};
        double chunk_t1_s{std::numeric_limits<double>::quiet_NaN()};
        int64_t chunk_t0_tick{0};
        int64_t chunk_t1_tick{0};
        OrbitPredictionProfileId profile_id{OrbitPredictionProfileId::Near};
    };

    struct OrbitPredictionPlannedChunkCacheEntry
    {
        OrbitPredictionPlannedChunkCacheKey key{};
        orbitsim::State start_state{};
        orbitsim::State end_state{};
        OrbitPredictionAdaptiveStageDiagnostics diagnostics{};
        std::vector<orbitsim::TrajectorySegment> segments{};
        std::vector<orbitsim::TrajectorySegment> seam_validation_segments{};
        std::vector<orbitsim::TrajectorySample> samples{};
        std::vector<OrbitPredictionManeuverNodePreview> previews{};
    };
} // namespace Game
