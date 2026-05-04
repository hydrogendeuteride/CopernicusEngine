#pragma once

#include "game/orbit/prediction/orbit_prediction_types.h"

#include "orbitsim/game_sim.hpp"
#include "orbitsim/trajectory_segments.hpp"
#include "orbitsim/trajectory_types.hpp"

#include <condition_variable>
#include <cassert>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
#include <vector>

namespace Game
{
    #if defined(VULKAN_ENGINE_GAMEPLAY_TEST_ACCESS)
    #define VULKAN_ENGINE_ORBIT_PREDICTION_SERVICE_PRIVATE public
    #else
    #define VULKAN_ENGINE_ORBIT_PREDICTION_SERVICE_PRIVATE private
    #endif

    class OrbitPredictionService
    {
    public:
        using SharedCelestialEphemeris = OrbitPredictionSharedCelestialEphemeris;
        using RequestKind = OrbitPredictionRequestKind;
        using SolveQuality = OrbitPredictionSolveQuality;
        using RequestPriority = OrbitPredictionRequestPriority;
        using Status = OrbitPredictionStatus;
        using AdaptiveStageDiagnostics = OrbitPredictionAdaptiveStageDiagnostics;
        using Diagnostics = OrbitPredictionDiagnostics;
        using ManeuverNodePreview = OrbitPredictionManeuverNodePreview;
        using ManeuverImpulse = OrbitPredictionManeuverImpulse;
        using PredictionProfileId = OrbitPredictionProfileId;
        using PredictionChunkBoundaryFlags = OrbitPredictionChunkBoundaryFlags;
        using ChunkQualityState = OrbitPredictionChunkQualityState;
        using PublishStage = OrbitPredictionPublishStage;
        using PredictionProfileDefinition = OrbitPredictionProfileDefinition;
        using PredictionChunkPlan = OrbitPredictionChunkPlan;
        using PublishedChunk = OrbitPredictionPublishedChunk;
        using StreamedPlannedChunk = OrbitPredictionStreamedPlannedChunk;
        using PredictionSolvePlan = OrbitPredictionSolvePlan;
        using ChunkActivityProbe = OrbitPredictionChunkActivityProbe;
        using ChunkSeamDiagnostics = OrbitPredictionChunkSeamDiagnostics;
        using Request = OrbitPredictionRequest;
        using Result = OrbitPredictionResult;
        using EphemerisSamplingSpec = OrbitPredictionEphemerisSamplingSpec;
        using EphemerisBuildRequest = OrbitPredictionEphemerisBuildRequest;
        using CachedEphemerisEntry = OrbitPredictionCachedEphemerisEntry;

        OrbitPredictionService();
        ~OrbitPredictionService();

        OrbitPredictionService(const OrbitPredictionService &) = delete;
        OrbitPredictionService &operator=(const OrbitPredictionService &) = delete;

        // Queue or replace the latest prediction job for a track; work runs on the background thread.
        // Returns the assigned generation so runtime can track request ordering.
        uint64_t request(Request request);
        // Mark maneuver-backed completed/published work stale before a replacement request is submitted.
        void invalidate_maneuver_plan_revision(uint64_t track_id, uint64_t maneuver_plan_revision);
        // Non-blocking poll for the next completed prediction result.
        std::optional<Result> poll_completed();
        // Invalidate queued/in-flight work and clear cached ephemerides.
        void reset();
        // Reuse a compatible cached ephemeris when possible, otherwise build and cache one.
        SharedCelestialEphemeris get_or_build_ephemeris(const EphemerisBuildRequest &request);
        // Derive spacecraft prediction horizon and cadence from the current orbital state.
        static EphemerisSamplingSpec build_ephemeris_sampling_spec(const Request &request);

        using PlannedChunkCacheKey = OrbitPredictionPlannedChunkCacheKey;
        using PlannedChunkCacheEntry = OrbitPredictionPlannedChunkCacheEntry;

    VULKAN_ENGINE_ORBIT_PREDICTION_SERVICE_PRIVATE:
        struct PlannedChunkCacheKeyHash
        {
            [[nodiscard]] std::size_t operator()(const PlannedChunkCacheKey &key) const noexcept
            {
                std::size_t seed = std::hash<uint64_t>{}(key.track_id);
                const auto combine = [&seed](const std::size_t value) {
                    seed ^= value + 0x9e3779b97f4a7c15ull + (seed << 6u) + (seed >> 2u);
                };

                combine(std::hash<uint64_t>{}(key.baseline_generation_id));
                combine(std::hash<uint64_t>{}(key.upstream_maneuver_hash));
                combine(std::hash<uint64_t>{}(key.frame_independent_generation));
                combine(std::hash<int64_t>{}(key.chunk_t0_tick));
                combine(std::hash<int64_t>{}(key.chunk_t1_tick));
                combine(std::hash<uint8_t>{}(static_cast<uint8_t>(key.profile_id)));
                return seed;
            }
        };

        struct PlannedChunkCacheKeyEqual
        {
            [[nodiscard]] bool operator()(const PlannedChunkCacheKey &a,
                                          const PlannedChunkCacheKey &b) const noexcept
            {
                return a.track_id == b.track_id &&
                       a.baseline_generation_id == b.baseline_generation_id &&
                       a.upstream_maneuver_hash == b.upstream_maneuver_hash &&
                       a.frame_independent_generation == b.frame_independent_generation &&
                       a.chunk_t0_tick == b.chunk_t0_tick &&
                       a.chunk_t1_tick == b.chunk_t1_tick &&
                       a.profile_id == b.profile_id;
            }
        };

        struct ReusableBaselineCacheEntry
        {
            uint64_t generation_id{0};
            uint64_t request_epoch{0};
            SharedCelestialEphemeris shared_ephemeris{};
            std::vector<orbitsim::TrajectorySample> trajectory_inertial{};
            std::vector<orbitsim::TrajectorySegment> trajectory_segments_inertial{};
        };

        struct PendingJob
        {
            uint64_t track_id{0};
            uint64_t request_epoch{0};
            uint64_t generation_id{0};
            Request request{};
        };

        struct PredictionRouteServiceAdapter;
        struct PredictionJobResultPublisher;
        struct PredictionJobRunner;

        // Execute a single queued prediction request on the worker and publish zero or more staged results.
        void compute_prediction(const PendingJob &job);
        std::optional<ReusableBaselineCacheEntry> find_reusable_baseline(uint64_t track_id, uint64_t request_epoch) const;
        void store_reusable_baseline(uint64_t track_id,
                                     uint64_t generation_id,
                                     uint64_t request_epoch,
                                     SharedCelestialEphemeris shared_ephemeris,
                                     std::vector<orbitsim::TrajectorySample> trajectory_inertial,
                                     std::vector<orbitsim::TrajectorySegment> trajectory_segments_inertial);
        std::optional<PlannedChunkCacheEntry> find_cached_planned_chunk(
                const PlannedChunkCacheKey &key,
                const orbitsim::State &expected_start_state);
        void store_cached_planned_chunk(PlannedChunkCacheEntry entry);
        bool publish_completed_result(const PendingJob &job, Result result);
        // Drop stale results after reset() or when a newer request supersedes the same track.
        static bool should_publish_result(const PendingJob &job,
                                          uint64_t current_request_epoch,
                                          const std::unordered_map<uint64_t, uint64_t> &latest_requested_generation_by_track);
        bool should_continue_job(uint64_t track_id,
                                 uint64_t generation_id,
                                 uint64_t request_epoch,
                                 uint64_t maneuver_plan_revision,
                                 SolveQuality solve_quality) const;
        // Background loop that consumes queued jobs and publishes fresh results.
        void worker_loop();
        SharedCelestialEphemeris get_or_build_ephemeris(const EphemerisBuildRequest &request,
                                                        const std::function<bool()> &cancel_requested,
                                                        AdaptiveStageDiagnostics *out_diagnostics = nullptr,
                                                        bool *out_cache_reused = nullptr);

        std::vector<std::thread> _workers;
        mutable std::mutex _mutex;
        std::condition_variable _cv;
        bool _running{true};

        std::deque<PendingJob> _pending_jobs{};
        std::deque<Result> _completed{};

        uint64_t _request_epoch{1};
        uint64_t _next_generation_id{1};
        std::unordered_map<uint64_t, uint64_t> _latest_requested_generation_by_track{};
        std::unordered_map<uint64_t, uint64_t> _latest_maneuver_plan_revision_by_track{};

        mutable std::mutex _baseline_cache_mutex;
        std::unordered_map<uint64_t, ReusableBaselineCacheEntry> _reusable_baseline_by_track{};

        mutable std::mutex _planned_chunk_cache_mutex;
        std::list<PlannedChunkCacheEntry> _planned_chunk_cache{};
        std::unordered_map<PlannedChunkCacheKey,
                           std::list<PlannedChunkCacheEntry>::iterator,
                           PlannedChunkCacheKeyHash,
                           PlannedChunkCacheKeyEqual>
                _planned_chunk_cache_by_key{};

        std::mutex _ephemeris_mutex;
        std::vector<CachedEphemerisEntry> _ephemeris_cache{};
        uint64_t _next_ephemeris_use_serial{1};
    };

    #undef VULKAN_ENGINE_ORBIT_PREDICTION_SERVICE_PRIVATE
} // namespace Game
