#pragma once

#include "game/orbit/kepler/kepler_celestial_nbody.h"
#include "game/orbit/kepler/kepler_prediction_options.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_builder.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_draw.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_state.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_subject.h"

#include <algorithm>
#include <cmath>
#include <span>
#include <vector>

namespace Game
{
    // Frame inputs collected by GameplayState.
    struct KeplerPredictionUpdateContext
    {
        const OrbitalRuntimeSystem *orbit{nullptr};
        const GameWorld *world{nullptr};
        const Physics::PhysicsWorld *physics{nullptr};
        const Physics::PhysicsContext *physics_context{nullptr};
        const ScenarioConfig *scenario_config{nullptr};
        bool enabled{true};
        double current_sim_time_s{0.0};
        double current_wall_time_s{0.0};
        bool rails_warp_active{false};
        double requested_horizon_s{0.0};
        orbitsim::BodyId fixed_primary_body_id{orbitsim::kInvalidBodyId};
        KeplerPredictionOptions options{};
        KeplerArcLineOptions line_options{};
        std::span<const KeplerManeuverNode> maneuver_nodes{};
        uint64_t maneuver_revision{0};
        bool build_celestial_kepler_tracks{false};
        bool build_celestial_nbody_tracks{true};
    };

    inline KeplerPropagationOptionsFingerprint make_kepler_propagation_fingerprint(
            const orbitsim::KeplerPropagationOptions &options)
    {
        return KeplerPropagationOptionsFingerprint{
                .max_iterations = options.max_iterations,
                .abs_tolerance = options.abs_tolerance,
                .rel_tolerance = options.rel_tolerance,
                .step_tolerance = options.step_tolerance,
                .rel_step_tolerance = options.rel_step_tolerance,
                .near_parabolic_alpha_tolerance = options.near_parabolic_alpha_tolerance,
                .max_hyperbolic_stumpff_arg = options.max_hyperbolic_stumpff_arg,
        };
    }

    inline KeplerSoiSwitchOptionsFingerprint make_kepler_soi_switch_fingerprint(
            const orbitsim::SoiSwitchOptions &options)
    {
        return KeplerSoiSwitchOptionsFingerprint{
                .enter_scale = options.enter_scale,
                .exit_scale = options.exit_scale,
                .prefer_smallest_soi = options.prefer_smallest_soi,
                .fallback_to_max_accel = options.fallback_to_max_accel,
        };
    }

    inline KeplerCelestialNBodyEphemerisFingerprint make_kepler_ephemeris_fingerprint(
            const KeplerCelestialNBodyEphemerisOptions &options)
    {
        return KeplerCelestialNBodyEphemerisFingerprint{
                .min_dt_s = options.min_dt_s,
                .max_dt_s = options.max_dt_s,
                .soft_max_segments = options.soft_max_segments,
                .hard_max_segments = options.hard_max_segments,
                .pos_tolerance_m = options.pos_tolerance_m,
                .vel_tolerance_mps = options.vel_tolerance_mps,
                .relative_tolerance_floor = options.relative_tolerance_floor,
        };
    }

    inline KeplerPatchedConicsFingerprint make_kepler_patched_conics_fingerprint(
            const KeplerPatchedConicsOptions &options)
    {
        return KeplerPatchedConicsFingerprint{
                .enabled = options.enabled,
                .max_patches = options.max_patches,
                .max_search_step_s = options.max_search_step_s,
                .refine_tolerance_s = options.refine_tolerance_s,
                .min_patch_duration_s = options.min_patch_duration_s,
        };
    }

    inline KeplerPredictionInputFingerprint make_kepler_prediction_input_fingerprint(
            const KeplerPredictionUpdateContext &context,
            const KeplerWorldFrame &world_frame)
    {
        return KeplerPredictionInputFingerprint{
                .requested_horizon_s = context.requested_horizon_s,
                .fixed_primary_body_id = context.fixed_primary_body_id,
                .world_reference_body_id = world_frame.world_reference_body_id,
                .maneuver_revision = context.maneuver_revision,
                .build_celestial_kepler_tracks = context.build_celestial_kepler_tracks,
                .build_celestial_nbody_tracks = context.build_celestial_nbody_tracks,

                .elliptic_period_count = context.options.elliptic_period_count,
                .open_orbit_window_s = context.options.open_orbit_window_s,
                .fallback_primary_hysteresis_keep_ratio =
                        context.options.fallback_primary_hysteresis_keep_ratio,
                .celestial_nbody_horizon_cap_s = context.options.celestial_nbody_horizon_cap_s,
                .celestial_line_max_time_step_s = context.options.celestial_line_max_time_step_s,
                .celestial_line_max_vertices_per_track =
                        context.options.celestial_line_max_vertices_per_track,
                .primary_switch = make_kepler_soi_switch_fingerprint(context.options.primary_switch),
                .prediction_propagation = make_kepler_propagation_fingerprint(context.options.propagation),
                .celestial_nbody_ephemeris =
                        make_kepler_ephemeris_fingerprint(context.options.celestial_nbody_ephemeris),
                .patched_conics =
                        make_kepler_patched_conics_fingerprint(context.options.patched_conics),

                .line_max_time_step_s = context.line_options.max_time_step_s,
                .line_min_time_step_s = context.line_options.min_time_step_s,
                .line_max_chord_error_m = context.line_options.max_chord_error_m,
                .line_max_vertices_per_arc = context.line_options.max_vertices_per_arc,
                .line_max_vertices_total = context.line_options.max_vertices_total,
                .line_include_start = context.line_options.include_start,
                .line_include_end = context.line_options.include_end,
                .line_propagation = make_kepler_propagation_fingerprint(context.line_options.propagation),
        };
    }

    inline double kepler_prediction_cache_reuse_horizon_s(const double required_horizon_s)
    {
        constexpr double kMinReuseS = 10.0 * 60.0;
        constexpr double kMaxReuseS = 7.0 * 24.0 * 60.0 * 60.0;
        if (!(required_horizon_s > 0.0) || !std::isfinite(required_horizon_s))
        {
            return kMinReuseS;
        }
        return std::clamp(required_horizon_s * 0.25, kMinReuseS, kMaxReuseS);
    }

    inline double kepler_prediction_build_horizon_s(const double required_horizon_s,
                                                    const double horizon_cap_s)
    {
        if (!(required_horizon_s > 0.0) || !std::isfinite(required_horizon_s))
        {
            return required_horizon_s;
        }

        double build_horizon_s =
                required_horizon_s + kepler_prediction_cache_reuse_horizon_s(required_horizon_s);
        if (std::isfinite(horizon_cap_s) && horizon_cap_s > 0.0)
        {
            build_horizon_s = std::min(build_horizon_s, horizon_cap_s);
        }
        return build_horizon_s;
    }

    // Owns Kepler prediction state, caching, update, and draw dispatch.
    class KeplerPredictionSystem
    {
    public:
        KeplerPredictionState &state() { return _state; }
        const KeplerPredictionState &state() const { return _state; }

        void reset();
        void mark_dirty();
        void update(const KeplerPredictionUpdateContext &context);
        void draw(const KeplerPredictionDrawContext &context) const;

    private:
        // Reusable celestial n-body ephemeris cache.
        struct CelestialNBodyEphemerisCache
        {
            bool valid{false};
            const orbitsim::GameSimulation *simulation{nullptr};
            KeplerSharedCelestialEphemeris ephemeris{};
            KeplerBodyStateProvider body_state_provider{};
            double t0_s{0.0};
            double t_end_s{0.0};
            double uncapped_required_horizon_s{0.0};
            double required_horizon_s{0.0};
            double built_horizon_s{0.0};
            double horizon_cap_s{0.0};
            bool horizon_capped{false};
            double gravitational_constant{0.0};
            double softening_length_m{0.0};
            KeplerCelestialNBodyEphemerisOptions ephemeris_options{};
            orbitsim::BodyId world_reference_body_id{orbitsim::kInvalidBodyId};
            std::vector<orbitsim::BodyId> body_ids{};
            std::vector<double> body_masses_kg{};
            KeplerOrbitStatus status{KeplerOrbitStatus::InvalidInput};
            orbitsim::AdaptiveEphemerisDiagnostics diagnostics{};
        };

        // Reuses or rebuilds the celestial ephemeris cache.
        const CelestialNBodyEphemerisCache &resolve_celestial_nbody_cache(
                const KeplerPredictionUpdateContext &context,
                const KeplerWorldFrame &world_frame,
                double required_horizon_s,
                double uncapped_required_horizon_s,
                double horizon_cap_s,
                bool horizon_capped);

        KeplerPredictionState _state{};
        CelestialNBodyEphemerisCache _celestial_nbody_cache{};
        mutable std::vector<Picking::LinePickSegmentData> _pick_segment_scratch{};
    };
} // namespace Game
