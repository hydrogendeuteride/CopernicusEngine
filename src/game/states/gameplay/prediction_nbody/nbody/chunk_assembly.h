#pragma once

#include "game/states/gameplay/prediction_nbody/gameplay_state_prediction_types.h"

#include <cstdint>
#include <functional>
#include <vector>

namespace Game
{
    struct StreamedChunkAssemblyBuilder
    {
        using CancelCheck = std::function<bool()>;

        static bool rebuild_from_streamed(
                PredictionChunkAssembly &out_assembly,
                const PredictionSolverTrajectoryCache &solver,
                const PredictionDisplayFrameCache &display,
                const std::vector<OrbitPredictionStreamedPlannedChunk> &streamed_chunks,
                uint64_t generation_id,
                const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
                const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
                const CancelCheck &cancel_requested = {},
                OrbitPredictionDerivedDiagnostics *diagnostics = nullptr,
                bool build_chunk_render_curves = false,
                bool use_dense_chunk_samples = true);

        static bool rebuild_from_streamed(
                PredictionChunkAssembly &out_assembly,
                const OrbitPredictionCache &cache,
                const std::vector<OrbitPredictionStreamedPlannedChunk> &streamed_chunks,
                uint64_t generation_id,
                const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
                const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
                const CancelCheck &cancel_requested = {},
                OrbitPredictionDerivedDiagnostics *diagnostics = nullptr,
                bool build_chunk_render_curves = false,
                bool use_dense_chunk_samples = true);

        static bool rebuild_from_published(
                PredictionChunkAssembly &out_assembly,
                const PredictionSolverTrajectoryCache &solver,
                const PredictionDisplayFrameCache &display,
                const std::vector<OrbitPredictionPublishedChunk> &published_chunks,
                uint64_t generation_id,
                const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
                const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
                const CancelCheck &cancel_requested = {},
                const std::vector<double> &node_times_s = {},
                OrbitPredictionDerivedDiagnostics *diagnostics = nullptr,
                bool build_chunk_render_curves = false,
                bool use_dense_chunk_samples = true);

        static bool rebuild_from_published(
                PredictionChunkAssembly &out_assembly,
                const OrbitPredictionCache &cache,
                const std::vector<OrbitPredictionPublishedChunk> &published_chunks,
                uint64_t generation_id,
                const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
                const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
                const CancelCheck &cancel_requested = {},
                const std::vector<double> &node_times_s = {},
                OrbitPredictionDerivedDiagnostics *diagnostics = nullptr,
                bool build_chunk_render_curves = false,
                bool use_dense_chunk_samples = true);
    };
} // namespace Game
