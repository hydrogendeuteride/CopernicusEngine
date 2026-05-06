#pragma once

#include "core/orbit_plot/orbit_plot.h"
#include "core/picking/line_pick_segment.h"
#include "core/world.h"
#include "game/orbit/nbody/types.h"
#include "game/orbit/orbit_render_curve.h"
#include "orbitsim/frame_spec.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace Game
{
    template<typename T>
    inline void prediction_hash_combine(uint64_t &seed, const T &value)
    {
        seed ^= static_cast<uint64_t>(std::hash<T>{}(value)) + 0x9e3779b97f4a7c15ULL + (seed << 6u) + (seed >> 2u);
    }

    inline uint64_t prediction_display_frame_key(const orbitsim::TrajectoryFrameSpec &spec)
    {
        uint64_t seed = 0xcbf29ce484222325ULL;
        prediction_hash_combine(seed, static_cast<uint32_t>(spec.type));
        prediction_hash_combine(seed, static_cast<uint32_t>(spec.primary_body_id));
        prediction_hash_combine(seed, static_cast<uint32_t>(spec.secondary_body_id));
        prediction_hash_combine(seed, static_cast<uint32_t>(spec.target_spacecraft_id));
        return seed;
    }

    enum class PredictionSubjectKind : uint8_t
    {
        None = 0,
        Orbiter,
        Celestial
    };

    struct PredictionSubjectKey
    {
        PredictionSubjectKind kind{PredictionSubjectKind::None};
        uint32_t value{0};

        [[nodiscard]] bool valid() const
        {
            return kind != PredictionSubjectKind::None && value != 0;
        }

        [[nodiscard]] uint64_t track_id() const
        {
            return (static_cast<uint64_t>(kind) << 32u) | static_cast<uint64_t>(value);
        }

        friend bool operator==(const PredictionSubjectKey &a, const PredictionSubjectKey &b)
        {
            return a.kind == b.kind && a.value == b.value;
        }

        friend bool operator!=(const PredictionSubjectKey &a, const PredictionSubjectKey &b)
        {
            return !(a == b);
        }
    };

    // Centralized style and tuning knobs for orbit prediction drawing.
    struct OrbitPredictionDrawPalette
    {
        glm::vec4 orbit_full{0.75f, 0.20f, 0.92f, 0.22f};
        glm::vec4 orbit_future{0.75f, 0.20f, 0.92f, 0.80f};
        glm::vec4 orbit_planned{1.00f, 0.62f, 0.10f, 0.90f};
        glm::vec4 velocity_ray{1.0f, 0.35f, 0.1f, 1.0f};
    };

    inline glm::vec4 prediction_overlay_seed_color(const std::size_t index)
    {
        static constexpr std::array<glm::vec4, 16> kOverlayColors{{
                {0.20f, 0.72f, 1.00f, 1.0f},
                {1.00f, 0.42f, 0.20f, 1.0f},
                {0.40f, 0.92f, 0.46f, 1.0f},
                {1.00f, 0.86f, 0.22f, 1.0f},
                {0.96f, 0.32f, 0.70f, 1.0f},
                {0.72f, 0.58f, 1.00f, 1.0f},
                {0.18f, 0.90f, 0.82f, 1.0f},
                {1.00f, 0.60f, 0.84f, 1.0f},
                {0.58f, 0.96f, 0.24f, 1.0f},
                {1.00f, 0.56f, 0.12f, 1.0f},
                {0.34f, 0.82f, 0.78f, 1.0f},
                {0.86f, 0.44f, 1.00f, 1.0f},
                {0.98f, 0.74f, 0.30f, 1.0f},
                {0.30f, 0.62f, 1.00f, 1.0f},
                {1.00f, 0.36f, 0.46f, 1.0f},
                {0.52f, 0.88f, 0.98f, 1.0f},
        }};
        return kOverlayColors[index % kOverlayColors.size()];
    }

    // Draw-time tuning shared by orbit plotting, dashed plans, and picking.
    struct OrbitPredictionDrawConfig
    {
        OrbitPredictionDrawPalette palette{};
        std::size_t pick_planned_reserve_segments{2'000};
        double pick_planned_reserve_ratio{0.25};
        double node_time_tolerance_s{1.0e-3};
        double dashed_segment_on_px{14.0};
        double dashed_segment_off_px{9.0};
        int dash_max_chunks_per_segment{128};
        int dash_max_chunks_per_window{4096};
        bool draw_planned_as_dashed{true};
    };

    enum class PredictionDerivedStatus : uint8_t
    {
        None = 0,
        Success,
        MissingSolverData,
        MissingEphemeris,
        FrameTransformFailed,
        FrameSamplesUnavailable,
        ContinuityFailed,
        Cancelled,
    };

    struct OrbitPredictionDerivedDiagnostics
    {
        PredictionDerivedStatus status{PredictionDerivedStatus::None};
        std::size_t frame_segment_count{0};
        std::size_t frame_segment_count_planned{0};
        std::size_t frame_sample_count{0};
        std::size_t frame_sample_count_planned{0};
        OrbitPredictionAdaptiveStageDiagnostics frame_base{};
        OrbitPredictionAdaptiveStageDiagnostics frame_planned{};
    };

    struct PredictionCacheIdentity
    {
        bool valid{false};
        uint64_t generation_id{0};
        uint64_t maneuver_plan_revision{0};
        bool maneuver_plan_signature_valid{false};
        uint64_t maneuver_plan_signature{0};
        double build_time_s{0.0};
        WorldVec3 build_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 build_vel_world{0.0, 0.0, 0.0};

        void clear()
        {
            valid = false;
            generation_id = 0;
            maneuver_plan_revision = 0;
            clear_maneuver_plan_signature();
            build_time_s = 0.0;
            build_pos_world = WorldVec3(0.0, 0.0, 0.0);
            build_vel_world = glm::dvec3(0.0, 0.0, 0.0);
        }

        void clear_maneuver_plan_signature()
        {
            maneuver_plan_signature_valid = false;
            maneuver_plan_signature = 0;
        }
    };

    struct PredictionTimeAnchorCache
    {
        uint64_t generation_id{0};
        uint64_t revision{std::numeric_limits<uint64_t>::max()};
        std::size_t source_count{std::numeric_limits<std::size_t>::max()};
        std::vector<double> times_s{};

        void clear()
        {
            generation_id = 0;
            revision = std::numeric_limits<uint64_t>::max();
            source_count = std::numeric_limits<std::size_t>::max();
            times_s.clear();
        }
    };

    struct PredictionSolverTrajectoryCache
    {
        using ManeuverNodePreview = OrbitPredictionManeuverNodePreview;
        using SharedSolverCoreData = OrbitPredictionResult::SharedCoreData;

        struct CorePayload
        {
            SharedSolverCoreData shared_solver_core_data{};
            OrbitPredictionSharedCelestialEphemeris shared_ephemeris{};
            std::vector<orbitsim::MassiveBody> massive_bodies;

            void clear()
            {
                shared_solver_core_data.reset();
                shared_ephemeris.reset();
                massive_bodies.clear();
            }

            void set_shared_solver_core_data(SharedSolverCoreData core_data)
            {
                shared_solver_core_data = std::move(core_data);
                if (shared_solver_core_data)
                {
                    shared_ephemeris = shared_solver_core_data->shared_ephemeris;
                }
            }
        };

        struct BasePayload
        {
            // Canonical prediction output always stays in orbitsim's inertial frame.
            std::vector<orbitsim::TrajectorySample> trajectory_inertial;
            std::vector<orbitsim::TrajectorySegment> trajectory_segments_inertial;

            void clear()
            {
                trajectory_inertial.clear();
                trajectory_segments_inertial.clear();
            }
        };

        struct PlannedPayload
        {
            std::vector<orbitsim::TrajectorySample> trajectory_inertial;
            std::vector<orbitsim::TrajectorySegment> trajectory_segments_inertial;
            std::vector<ManeuverNodePreview> maneuver_previews;

            void clear()
            {
                trajectory_inertial.clear();
                trajectory_segments_inertial.clear();
                maneuver_previews.clear();
            }

            void copy_from(const PlannedPayload &src)
            {
                trajectory_inertial = src.trajectory_inertial;
                trajectory_segments_inertial = src.trajectory_segments_inertial;
                maneuver_previews = src.maneuver_previews;
            }
        };

        CorePayload core{};
        BasePayload base{};
        PlannedPayload planned{};

        void clear()
        {
            core.clear();
            base.clear();
            clear_planned();
        }

        void clear_planned()
        {
            planned.clear();
        }

        void copy_planned_from(const PredictionSolverTrajectoryCache &src)
        {
            planned.copy_from(src.planned);
        }

        void set_shared_solver_core_data(SharedSolverCoreData core_data)
        {
            core.set_shared_solver_core_data(std::move(core_data));
        }

        [[nodiscard]] const OrbitPredictionSharedCelestialEphemeris &resolved_shared_ephemeris() const
        {
            return core.shared_solver_core_data ? core.shared_solver_core_data->shared_ephemeris : core.shared_ephemeris;
        }

        [[nodiscard]] const std::vector<orbitsim::MassiveBody> &resolved_massive_bodies() const
        {
            return core.shared_solver_core_data ? core.shared_solver_core_data->massive_bodies : core.massive_bodies;
        }

        [[nodiscard]] const std::vector<orbitsim::TrajectorySample> &resolved_trajectory_inertial() const
        {
            return core.shared_solver_core_data ? core.shared_solver_core_data->trajectory_inertial
                                               : base.trajectory_inertial;
        }

        [[nodiscard]] const std::vector<orbitsim::TrajectorySegment> &resolved_trajectory_segments_inertial() const
        {
            return core.shared_solver_core_data ? core.shared_solver_core_data->trajectory_segments_inertial
                                               : base.trajectory_segments_inertial;
        }
    };

    struct OrbitChunk
    {
        uint32_t chunk_id{0};
        uint64_t generation_id{0};
        OrbitPredictionChunkQualityState quality_state{OrbitPredictionChunkQualityState::Final};
        double t0_s{std::numeric_limits<double>::quiet_NaN()};
        double t1_s{std::numeric_limits<double>::quiet_NaN()};
        std::vector<orbitsim::TrajectorySample> frame_samples{};
        std::vector<orbitsim::TrajectorySegment> frame_segments{};
        OrbitRenderCurve render_curve{};
        bool valid{false};

        [[nodiscard]] bool drawable() const
        {
            return valid &&
                   (frame_samples.size() >= 2 ||
                    !frame_segments.empty() ||
                    !render_curve.empty());
        }
    };

    struct PredictionChunkAssembly
    {
        bool valid{false};
        uint64_t generation_id{0};
        std::vector<OrbitChunk> chunks{};

        [[nodiscard]] bool drawable() const
        {
            return valid &&
                   std::any_of(chunks.begin(), chunks.end(), [](const OrbitChunk &chunk) {
                       return chunk.drawable();
                   });
        }

        [[nodiscard]] std::size_t segment_count() const
        {
            std::size_t count = 0;
            for (const OrbitChunk &chunk : chunks)
            {
                count += chunk.frame_segments.size();
            }
            return count;
        }

        [[nodiscard]] std::size_t sample_count() const
        {
            std::size_t count = 0;
            for (const OrbitChunk &chunk : chunks)
            {
                count += chunk.frame_samples.size();
            }
            return count;
        }

        [[nodiscard]] bool time_span(double &out_t0_s, double &out_t1_s) const
        {
            out_t0_s = std::numeric_limits<double>::quiet_NaN();
            out_t1_s = std::numeric_limits<double>::quiet_NaN();
            if (!valid)
            {
                return false;
            }

            for (const OrbitChunk &chunk : chunks)
            {
                if (!std::isfinite(chunk.t0_s) || !std::isfinite(chunk.t1_s) || !(chunk.t1_s > chunk.t0_s))
                {
                    continue;
                }
                out_t0_s = std::isfinite(out_t0_s) ? std::min(out_t0_s, chunk.t0_s) : chunk.t0_s;
                out_t1_s = std::isfinite(out_t1_s) ? std::max(out_t1_s, chunk.t1_s) : chunk.t1_s;
            }

            return std::isfinite(out_t0_s) && std::isfinite(out_t1_s) && out_t1_s > out_t0_s;
        }

        void clear()
        {
            valid = false;
            generation_id = 0;
            chunks.clear();
        }
    };

    struct PredictionDisplayFrameCache
    {
        std::vector<orbitsim::TrajectorySample> trajectory_frame;
        std::vector<orbitsim::TrajectorySegment> trajectory_segments_frame;
        OrbitRenderCurve render_curve_frame;
        PredictionChunkAssembly planned_chunk_assembly{};
        orbitsim::TrajectoryFrameSpec resolved_frame_spec{};
        uint64_t display_frame_key{0};
        uint64_t display_frame_revision{0};
        bool resolved_frame_spec_valid{false};

        void clear()
        {
            trajectory_frame.clear();
            trajectory_segments_frame.clear();
            render_curve_frame.clear();
            clear_planned();
            resolved_frame_spec = {};
            display_frame_key = 0;
            display_frame_revision = 0;
            resolved_frame_spec_valid = false;
        }

        void clear_planned()
        {
            planned_chunk_assembly.clear();
        }

        void copy_planned_from(const PredictionDisplayFrameCache &src)
        {
            planned_chunk_assembly = src.planned_chunk_assembly;
        }

        [[nodiscard]] bool has_planned_draw_data() const
        {
            return planned_chunk_assembly.drawable();
        }
    };

    struct PredictionAnalysisCache
    {
        std::vector<orbitsim::TrajectorySample> trajectory_analysis_bci;
        std::vector<orbitsim::TrajectorySegment> trajectory_segments_analysis_bci;
        orbitsim::BodyId analysis_cache_body_id{orbitsim::kInvalidBodyId};
        bool analysis_cache_valid{false};

        std::vector<float> altitude_km;
        std::vector<float> speed_kmps;

        // Classical orbital-element-like values for HUD.
        double semi_major_axis_m{0.0};
        double eccentricity{0.0};
        double orbital_period_s{0.0};
        double periapsis_alt_km{0.0};
        double apoapsis_alt_km{std::numeric_limits<double>::infinity()};
        orbitsim::BodyId metrics_body_id{orbitsim::kInvalidBodyId};
        bool metrics_valid{false};

        void clear()
        {
            trajectory_analysis_bci.clear();
            trajectory_segments_analysis_bci.clear();
            analysis_cache_body_id = orbitsim::kInvalidBodyId;
            analysis_cache_valid = false;
            clear_metrics(orbitsim::kInvalidBodyId, false);
        }

        void clear_metrics(const orbitsim::BodyId body_id, const bool valid_after_clear = true)
        {
            altitude_km.clear();
            speed_kmps.clear();
            semi_major_axis_m = 0.0;
            eccentricity = 0.0;
            orbital_period_s = 0.0;
            periapsis_alt_km = 0.0;
            apoapsis_alt_km = std::numeric_limits<double>::infinity();
            metrics_body_id = body_id;
            metrics_valid = valid_after_clear;
        }
    };

    struct OrbitPredictionCache
    {
        PredictionCacheIdentity identity{};
        PredictionSolverTrajectoryCache solver{};
        PredictionDisplayFrameCache display{};
        PredictionAnalysisCache analysis{};

        void set_shared_solver_core_data(PredictionSolverTrajectoryCache::SharedSolverCoreData core_data)
        {
            solver.set_shared_solver_core_data(std::move(core_data));
        }

        [[nodiscard]] const OrbitPredictionSharedCelestialEphemeris &resolved_shared_ephemeris() const
        {
            return solver.resolved_shared_ephemeris();
        }

        [[nodiscard]] const std::vector<orbitsim::MassiveBody> &resolved_massive_bodies() const
        {
            return solver.resolved_massive_bodies();
        }

        [[nodiscard]] const std::vector<orbitsim::TrajectorySample> &resolved_trajectory_inertial() const
        {
            return solver.resolved_trajectory_inertial();
        }

        [[nodiscard]] const std::vector<orbitsim::TrajectorySegment> &resolved_trajectory_segments_inertial() const
        {
            return solver.resolved_trajectory_segments_inertial();
        }

        [[nodiscard]] bool has_planned_frame_draw_data() const
        {
            return display.has_planned_draw_data();
        }

        // Reset every cached prediction artifact so the next update rebuilds from scratch.
        void clear()
        {
            identity.clear();
            solver.clear();
            display.clear();
            analysis.clear();
        }
    };

    inline void clear_prediction_cache_planned_data(OrbitPredictionCache &cache)
    {
        cache.solver.clear_planned();
        cache.display.clear_planned();
        cache.identity.clear_maneuver_plan_signature();
    }

    inline void copy_prediction_cache_planned_data(OrbitPredictionCache &dst, const OrbitPredictionCache &src)
    {
        dst.solver.copy_planned_from(src.solver);
        dst.display.copy_planned_from(src.display);
        dst.identity.maneuver_plan_signature_valid = src.identity.maneuver_plan_signature_valid;
        dst.identity.maneuver_plan_signature = src.identity.maneuver_plan_signature;
    }

    struct PredictionPreviewOverlay
    {
        PredictionChunkAssembly chunk_assembly{};

        void clear()
        {
            chunk_assembly.clear();
        }
    };

    struct PredictionFrameBoundChunkOverlay
    {
        PredictionChunkAssembly chunk_assembly{};
        uint64_t display_frame_key{0};
        uint64_t display_frame_revision{0};

        [[nodiscard]] bool matches_generation(uint64_t generation_id,
                                              uint64_t frame_key,
                                              uint64_t frame_revision) const
        {
            return chunk_assembly.generation_id == generation_id &&
                   display_frame_key == frame_key &&
                   display_frame_revision == frame_revision;
        }

        [[nodiscard]] bool ready_for_draw(uint64_t generation_id,
                                          uint64_t frame_key,
                                          uint64_t frame_revision) const
        {
            return chunk_assembly.valid &&
                   !chunk_assembly.chunks.empty() &&
                   matches_generation(generation_id, frame_key, frame_revision);
        }

        void reset_for_generation(uint64_t generation_id,
                                  uint64_t frame_key,
                                  uint64_t frame_revision)
        {
            chunk_assembly.clear();
            chunk_assembly.generation_id = generation_id;
            display_frame_key = frame_key;
            display_frame_revision = frame_revision;
        }

        void clear()
        {
            chunk_assembly.clear();
            display_frame_key = 0;
            display_frame_revision = 0;
        }
    };

    struct PredictionLinePickCache
    {
        uint64_t generation_id{0};
        uint64_t display_frame_key{0};
        uint64_t display_frame_revision{0};
        bool base_valid{false};
        bool planned_valid{false};
        WorldVec3 ref_body_world{0.0, 0.0, 0.0};
        WorldVec3 align_delta_world{0.0, 0.0, 0.0};
        glm::dmat3 frame_to_world{1.0};
        bool pick_frustum_valid{false};
        glm::mat4 pick_frustum_viewproj{1.0f};
        WorldVec3 pick_frustum_origin_world{0.0, 0.0, 0.0};
        double pick_frustum_margin_ratio{0.05};
        glm::dvec3 camera_world{0.0, 0.0, 0.0};
        double tan_half_fov{0.0};
        double viewport_height_px{1.0};
        double render_error_px{0.75};
        double base_t0_s{std::numeric_limits<double>::quiet_NaN()};
        double base_t1_s{std::numeric_limits<double>::quiet_NaN()};
        double planned_t0_s{std::numeric_limits<double>::quiet_NaN()};
        double planned_t1_s{std::numeric_limits<double>::quiet_NaN()};
        std::size_t base_max_segments{0};
        std::size_t planned_max_segments{0};
        bool base_use_adaptive_curve{false};
        bool planned_use_adaptive_curve{false};
        std::vector<Picking::LinePickSegmentData> base_segments;
        std::vector<Picking::LinePickSegmentData> planned_segments;

        void clear()
        {
            generation_id = 0;
            display_frame_key = 0;
            display_frame_revision = 0;
            base_valid = false;
            planned_valid = false;
            ref_body_world = WorldVec3(0.0, 0.0, 0.0);
            align_delta_world = WorldVec3(0.0, 0.0, 0.0);
            frame_to_world = glm::dmat3(1.0);
            pick_frustum_valid = false;
            pick_frustum_viewproj = glm::mat4(1.0f);
            pick_frustum_origin_world = WorldVec3(0.0, 0.0, 0.0);
            pick_frustum_margin_ratio = 0.05;
            camera_world = glm::dvec3(0.0, 0.0, 0.0);
            tan_half_fov = 0.0;
            viewport_height_px = 1.0;
            render_error_px = 0.75;
            base_t0_s = std::numeric_limits<double>::quiet_NaN();
            base_t1_s = std::numeric_limits<double>::quiet_NaN();
            planned_t0_s = std::numeric_limits<double>::quiet_NaN();
            planned_t1_s = std::numeric_limits<double>::quiet_NaN();
            base_max_segments = 0;
            planned_max_segments = 0;
            base_use_adaptive_curve = false;
            planned_use_adaptive_curve = false;
            base_segments.clear();
            planned_segments.clear();
        }
    };

    struct PredictionRenderLinePacketCache
    {
        bool valid{false};
        uint64_t generation_id{0};
        uint64_t display_frame_key{0};
        uint64_t display_frame_revision{0};
        uint64_t maneuver_plan_revision{0};
        uint64_t maneuver_plan_signature{0};
        uint64_t anchor_hash{0};
        WorldVec3 line_origin_world{0.0, 0.0, 0.0};
        WorldVec3 ref_body_world{0.0, 0.0, 0.0};
        WorldVec3 align_delta_world{0.0, 0.0, 0.0};
        glm::dmat3 frame_to_world{1.0};
        bool render_frustum_valid{false};
        glm::mat4 render_frustum_viewproj{1.0f};
        WorldVec3 render_frustum_origin_world{0.0, 0.0, 0.0};
        glm::dvec3 camera_world{0.0, 0.0, 0.0};
        double tan_half_fov{0.0};
        double viewport_height_px{1.0};
        double render_error_px{0.75};
        double selection_error_scale{1.0};
        double t0_s{std::numeric_limits<double>::quiet_NaN()};
        double t1_s{std::numeric_limits<double>::quiet_NaN()};
        std::size_t render_max_segments{0};
        float line_overlay_boost{0.0f};
        glm::vec4 color{1.0f};
        bool cap_hit{false};
        std::vector<OrbitPlotSystem::LineCommand> lines{};

        void clear()
        {
            valid = false;
            generation_id = 0;
            display_frame_key = 0;
            display_frame_revision = 0;
            maneuver_plan_revision = 0;
            maneuver_plan_signature = 0;
            anchor_hash = 0;
            line_origin_world = WorldVec3(0.0, 0.0, 0.0);
            ref_body_world = WorldVec3(0.0, 0.0, 0.0);
            align_delta_world = WorldVec3(0.0, 0.0, 0.0);
            frame_to_world = glm::dmat3(1.0);
            render_frustum_valid = false;
            render_frustum_viewproj = glm::mat4(1.0f);
            render_frustum_origin_world = WorldVec3(0.0, 0.0, 0.0);
            camera_world = glm::dvec3(0.0, 0.0, 0.0);
            tan_half_fov = 0.0;
            viewport_height_px = 1.0;
            render_error_px = 0.75;
            selection_error_scale = 1.0;
            t0_s = std::numeric_limits<double>::quiet_NaN();
            t1_s = std::numeric_limits<double>::quiet_NaN();
            render_max_segments = 0;
            line_overlay_boost = 0.0f;
            color = glm::vec4(1.0f);
            cap_hit = false;
            lines.clear();
        }
    };

    struct PredictionFrameOption
    {
        orbitsim::TrajectoryFrameSpec spec{};
        std::string label{};
    };

    struct PredictionFrameSelectionState
    {
        orbitsim::TrajectoryFrameSpec spec = orbitsim::TrajectoryFrameSpec::inertial();
        std::vector<PredictionFrameOption> options{};
        int selected_index{-1};

        void clear()
        {
            spec = orbitsim::TrajectoryFrameSpec::inertial();
            options.clear();
            selected_index = -1;
        }
    };

    enum class PredictionAnalysisMode : uint8_t
    {
        AutoPrimaryBCI = 0,
        FixedBodyBCI,
    };

    struct PredictionAnalysisSpec
    {
        PredictionAnalysisMode mode{PredictionAnalysisMode::AutoPrimaryBCI};
        orbitsim::BodyId fixed_body_id{orbitsim::kInvalidBodyId};
    };

    struct PredictionAnalysisOption
    {
        PredictionAnalysisSpec spec{};
        std::string label{};
    };

    struct PredictionAnalysisSelectionState
    {
        PredictionAnalysisSpec spec{};
        std::vector<PredictionAnalysisOption> options{};
        int selected_index{-1};

        void clear()
        {
            spec = {};
            options.clear();
            selected_index = -1;
        }
    };

    enum class PredictionTimeAnchorKind : uint8_t
    {
        None = 0,
        SimNow,
        SelectedNode,
        FirstFutureNode,
        FirstRelevantNode,
    };

    struct PredictionTimeContext
    {
        double sim_now_s{std::numeric_limits<double>::quiet_NaN()};
        double trajectory_t0_s{std::numeric_limits<double>::quiet_NaN()};
        double trajectory_t1_s{std::numeric_limits<double>::quiet_NaN()};
        double selected_node_time_s{std::numeric_limits<double>::quiet_NaN()};
        double first_future_node_time_s{std::numeric_limits<double>::quiet_NaN()};
        double first_relevant_node_time_s{std::numeric_limits<double>::quiet_NaN()};
        double last_future_node_time_s{std::numeric_limits<double>::quiet_NaN()};
        bool has_plan{false};
    };

    struct PredictionWindowPolicyResult
    {
        bool valid{false};
        double request_window_s{0.0};
        double visual_window_s{0.0};
        double pick_window_s{0.0};
        double exact_window_s{0.0};
        double visual_window_start_time_s{std::numeric_limits<double>::quiet_NaN()};
        double visual_window_end_time_s{std::numeric_limits<double>::quiet_NaN()};
        double pick_window_start_time_s{std::numeric_limits<double>::quiet_NaN()};
        double pick_window_end_time_s{std::numeric_limits<double>::quiet_NaN()};
        double visual_anchor_time_s{std::numeric_limits<double>::quiet_NaN()};
        double pick_anchor_time_s{std::numeric_limits<double>::quiet_NaN()};
        double exact_anchor_time_s{std::numeric_limits<double>::quiet_NaN()};
        PredictionTimeAnchorKind visual_anchor_kind{PredictionTimeAnchorKind::None};
        PredictionTimeAnchorKind pick_anchor_kind{PredictionTimeAnchorKind::None};
        PredictionTimeAnchorKind exact_anchor_kind{PredictionTimeAnchorKind::None};
        bool visual_anchor_is_future{false};
        bool pick_anchor_is_future{false};
        bool exact_anchor_is_future{false};
    };

    enum class PredictionPreviewRuntimeState : uint8_t
    {
        Idle = 0,
        EnterDrag,
        DragPreviewPending,
        PreviewStreaming,
        AwaitFullRefine,
    };

    // High-level runtime lifecycle derived from the existing preview/pending/cache fields.
    // This sits above the current bool/overlay combination so transition logic can move toward
    // a single explicit state machine without changing behavior immediately.
    enum class PredictionTrackLifecycleState : uint8_t
    {
        Idle = 0,
        NeedsRebuild,
        DragPreviewPending,
        PreviewStreaming,
        AwaitFullRefine,
        FullSolvePending,
        FullStreaming,
        FinalDerivedPending,
        Stable,
    };

    struct PredictionPreviewAnchor
    {
        bool valid{false};
        int anchor_node_id{-1};
        double anchor_time_s{std::numeric_limits<double>::quiet_NaN()};
        double request_window_s{0.0};
        double visual_window_s{0.0};
        double exact_window_s{0.0};
    };

    struct PredictionDragDebugTelemetry
    {
        using Clock = std::chrono::steady_clock;
        using TimePoint = Clock::time_point;

        uint64_t drag_session_id{0};
        uint64_t drag_update_count{0};
        uint64_t request_count{0};
        uint64_t full_request_count{0};
        uint64_t fast_preview_request_count{0};
        uint64_t solver_result_count{0};
        uint64_t full_solver_result_count{0};
        uint64_t fast_preview_solver_result_count{0};
        uint64_t derived_result_count{0};
        uint64_t publish_count{0};
        uint64_t last_request_generation_id{0};
        uint64_t last_solver_result_generation_id{0};
        uint64_t last_derived_result_generation_id{0};

        TimePoint drag_started_tp{};
        TimePoint last_drag_update_tp{};
        TimePoint last_drag_end_tp{};
        TimePoint last_preview_dirty_tp{};
        TimePoint last_request_tp{};
        TimePoint last_solver_result_tp{};
        TimePoint last_derived_result_tp{};
        TimePoint last_publish_tp{};

        double drag_to_request_ms_last{0.0};
        double drag_to_request_ms_peak{0.0};
        double drag_apply_ms_last{0.0};
        double drag_apply_ms_peak{0.0};
        double request_to_solver_ms_last{0.0};
        double request_to_solver_ms_peak{0.0};
        double request_to_derived_ms_last{0.0};
        double request_to_derived_ms_peak{0.0};
        double solver_to_derived_ms_last{0.0};
        double solver_to_derived_ms_peak{0.0};
        double derived_worker_ms_last{0.0};
        double derived_worker_ms_peak{0.0};
        double derived_frame_build_ms_last{0.0};
        double derived_chunk_assembly_ms_last{0.0};
        double derived_apply_ms_last{0.0};
        double derived_apply_ms_peak{0.0};

        std::size_t chunk_planned_segments_last{0};
        std::size_t chunk_planned_samples_last{0};

        OrbitPredictionSolveQuality last_request_solve_quality{OrbitPredictionSolveQuality::Full};
        OrbitPredictionSolveQuality last_result_solve_quality{OrbitPredictionSolveQuality::Full};
        double last_request_future_window_s{0.0};
        std::size_t last_request_maneuver_count{0};
        bool last_request_preview_patch{false};
        bool last_request_full_stream_publish{false};
        bool drag_active{false};

        static bool has_time(const TimePoint &tp)
        {
            return tp.time_since_epoch() != Clock::duration::zero();
        }

        void clear()
        {
            *this = {};
        }
    };

    struct PredictionTrackState
    {
        PredictionSubjectKey key{};
        std::string label{};
        OrbitPredictionCache cache{};
        OrbitPredictionCache authoritative_cache{};
        PredictionLinePickCache pick_cache{};
        bool dirty{true};
        bool request_pending{false};
        bool derived_request_pending{false};
        uint64_t latest_requested_generation_id{0};
        uint64_t latest_requested_authoritative_generation_id{0};
        uint64_t latest_requested_derived_generation_id{0};
        uint64_t latest_requested_derived_display_frame_key{0};
        uint64_t latest_requested_derived_display_frame_revision{0};
        orbitsim::BodyId latest_requested_derived_analysis_body_id{orbitsim::kInvalidBodyId};
        OrbitPredictionPublishStage latest_requested_derived_publish_stage{OrbitPredictionPublishStage::Final};
        OrbitPredictionSolveQuality pending_solve_quality{OrbitPredictionSolveQuality::Full};
        bool pending_solver_has_maneuver_plan{false};
        uint64_t pending_solver_plan_signature{0};
        bool pending_derived_has_maneuver_plan{false};
        uint64_t pending_derived_plan_signature{0};
        bool invalidated_while_pending{false};
        PredictionDragDebugTelemetry drag_debug{};
        bool supports_maneuvers{false};
        bool is_celestial{false};
        orbitsim::BodyId auto_primary_body_id{orbitsim::kInvalidBodyId};
        PredictionPreviewRuntimeState preview_state{PredictionPreviewRuntimeState::Idle};
        PredictionPreviewAnchor preview_anchor{};
        PredictionPreviewOverlay preview_overlay{};
        PredictionFrameBoundChunkOverlay full_stream_overlay{};
        PredictionTimeAnchorCache planned_curve_preview_time_cache{};
        PredictionTimeAnchorCache stale_planned_curve_preview_time_cache{};
        PredictionRenderLinePacketCache planned_render_line_cache{};
        PredictionRenderLinePacketCache stale_planned_render_line_cache{};
        double preview_entered_at_s{std::numeric_limits<double>::quiet_NaN()};
        double preview_last_anchor_refresh_at_s{std::numeric_limits<double>::quiet_NaN()};
        double preview_last_request_at_s{std::numeric_limits<double>::quiet_NaN()};
        double solver_ms_last{0.0};
        OrbitPredictionDiagnostics solver_diagnostics{};
        OrbitPredictionDerivedDiagnostics derived_diagnostics{};

        void clear_runtime()
        {
            cache.clear();
            authoritative_cache.clear();
            pick_cache.clear();
            dirty = true;
            request_pending = false;
            derived_request_pending = false;
            latest_requested_generation_id = 0;
            latest_requested_authoritative_generation_id = 0;
            latest_requested_derived_generation_id = 0;
            latest_requested_derived_display_frame_key = 0;
            latest_requested_derived_display_frame_revision = 0;
            latest_requested_derived_analysis_body_id = orbitsim::kInvalidBodyId;
            latest_requested_derived_publish_stage = OrbitPredictionPublishStage::Final;
            pending_solve_quality = OrbitPredictionSolveQuality::Full;
            pending_solver_has_maneuver_plan = false;
            pending_solver_plan_signature = 0;
            pending_derived_has_maneuver_plan = false;
            pending_derived_plan_signature = 0;
            invalidated_while_pending = false;
            drag_debug.clear();
            auto_primary_body_id = orbitsim::kInvalidBodyId;
            preview_state = PredictionPreviewRuntimeState::Idle;
            preview_anchor = {};
            preview_overlay.clear();
            full_stream_overlay.clear();
            planned_curve_preview_time_cache.clear();
            stale_planned_curve_preview_time_cache.clear();
            planned_render_line_cache.clear();
            stale_planned_render_line_cache.clear();
            preview_entered_at_s = std::numeric_limits<double>::quiet_NaN();
            preview_last_anchor_refresh_at_s = std::numeric_limits<double>::quiet_NaN();
            preview_last_request_at_s = std::numeric_limits<double>::quiet_NaN();
            solver_ms_last = 0.0;
            solver_diagnostics = {};
            derived_diagnostics = {};
        }
    };

    struct PredictionGroup
    {
        std::string name{};
        PredictionSubjectKey primary_subject{};
        std::vector<PredictionSubjectKey> members{};
    };

    struct PredictionSelectionState
    {
        PredictionSubjectKey active_subject{};
        std::vector<PredictionSubjectKey> overlay_subjects{};
        int selected_group_index{-1};

        void clear()
        {
            active_subject = {};
            overlay_subjects.clear();
            selected_group_index = -1;
        }
    };

    struct OrbitPlotPerfStats
    {
        double solver_ms_last{0.0};
        double render_lod_ms_last{0.0};
        double pick_lod_ms_last{0.0};

        uint32_t solver_segments_base{0};
        uint32_t solver_segments_planned{0};
        uint32_t pick_segments_before_cull{0};
        uint32_t pick_segments{0};

        bool render_cap_hit_last_frame{false};
        bool pick_cap_hit_last_frame{false};
        uint64_t render_cap_hits_total{0};
        uint64_t pick_cap_hits_total{0};

        // Debug: planned pick window values from last frame.
        bool planned_window_valid{false};
        double planned_window_t0p{0.0};
        double planned_window_now_s{0.0};
        double planned_window_anchor_s{0.0};
        double planned_window_t_start{0.0};
        double planned_window_t_end{0.0};

        // Chunk assembly draw stats.
        uint32_t planned_chunk_count{0};
        uint32_t planned_chunks_drawn{0};
        uint32_t planned_fallback_range_count{0};
        double planned_chunk_enqueue_ms_last{0.0};
        double planned_fallback_draw_ms_last{0.0};
    };
} // namespace Game
