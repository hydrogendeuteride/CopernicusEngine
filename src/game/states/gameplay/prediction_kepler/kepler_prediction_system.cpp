#include "game/states/gameplay/prediction_kepler/kepler_prediction_system.h"

#include "game/orbit/kepler/kepler_celestial_nbody.h"
#include "game/states/gameplay/orbital/orbital_runtime_system.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <span>
#include <utility>
#include <vector>

namespace Game
{
    namespace
    {
        using KeplerPerfClock = std::chrono::steady_clock;

        double elapsed_ms(const KeplerPerfClock::time_point start,
                          const KeplerPerfClock::time_point end = KeplerPerfClock::now())
        {
            return std::chrono::duration<double, std::milli>(end - start).count();
        }

        void record_track_perf(KeplerPredictionState::PerfDebug &perf,
                               const KeplerPredictionState::Track &track)
        {
            perf.total_base_arcs += track.base_arcs.size();
            perf.total_planned_arcs += track.planned_arcs.size();
            perf.total_prebuilt_line_vertices += track.prebuilt_base_lines.vertices.size();
            perf.total_prebuilt_line_requested_samples +=
                    track.prebuilt_base_lines.diagnostics.requested_samples;
            perf.total_base_patch_events += track.base_patch_events.size();
            perf.total_planned_patch_events += track.planned_patch_events.size();
            perf.total_orbit_ms += track.perf.orbit_ms;
            perf.total_base_patch_ms += track.perf.base_patch_ms;
            perf.total_planned_arc_ms += track.perf.planned_arc_ms;

            if (!track.active_player)
            {
                return;
            }

            perf.active_base_arcs += track.base_arcs.size();
            perf.active_planned_arcs += track.planned_arcs.size();
            perf.active_prebuilt_line_vertices += track.prebuilt_base_lines.vertices.size();
            perf.active_prebuilt_line_requested_samples +=
                    track.prebuilt_base_lines.diagnostics.requested_samples;
            perf.active_orbit_ms += track.perf.orbit_ms;
            perf.active_base_patch_ms += track.perf.base_patch_ms;
            perf.active_planned_arc_ms += track.perf.planned_arc_ms;
        }

        void record_perf_tracks(KeplerPredictionState::PerfDebug &perf,
                                const std::vector<KeplerPredictionState::Track> &tracks)
        {
            perf.track_count = tracks.size();
            for (const KeplerPredictionState::Track &track : tracks)
            {
                record_track_perf(perf, track);
            }
        }

        // Publishes an active-player or first-valid representative track.
        void publish_representative_track(KeplerPredictionState &state)
        {
            const KeplerPredictionState::Track *representative = nullptr;
            for (const KeplerPredictionState::Track &track : state.tracks)
            {
                if (track.valid && track.active_player)
                {
                    representative = &track;
                    break;
                }
            }
            if (!representative)
            {
                const auto it = std::find_if(state.tracks.begin(),
                                             state.tracks.end(),
                                             [](const KeplerPredictionState::Track &track) {
                                                 return track.valid;
                                             });
                if (it != state.tracks.end())
                {
                    representative = &(*it);
                }
            }

            if (!representative)
            {
                state.clear_result(KeplerOrbitStatus::InvalidSubjectState);
                return;
            }

            state.valid = representative->valid;
            state.status = representative->status;
            state.horizon_s = representative->horizon_s;
            state.primary_body_id = representative->primary_body_id;
        }

        // Guards Kepler math against invalid vectors.
        bool finite_vec3(const orbitsim::Vec3 &v)
        {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        }

        // Keeps subject identity even when prediction fails.
        KeplerPredictionState::Track make_invalid_track(const KeplerPredictionSubject &subject,
                                                        const KeplerOrbitStatus status)
        {
            KeplerPredictionState::Track track{};
            track.valid = false;
            track.celestial = subject.celestial;
            track.active_player = subject.active_player;
            track.entity = subject.entity;
            track.body_id = subject.body_id;
            track.label = subject.label;
            track.orbit_rgb = subject.orbit_rgb;
            track.status = status;
            return track;
        }

        KeplerPredictionState::Track make_invalid_celestial_nbody_track(
                const KeplerPredictionSubject &subject,
                const KeplerOrbitStatus status)
        {
            KeplerPredictionState::Track track = make_invalid_track(subject, status);
            track.celestial_nbody = true;
            return track;
        }

        KeplerArcLineOptions make_celestial_track_line_options(
                const KeplerArcLineOptions &base,
                const KeplerPredictionOptions &options)
        {
            KeplerArcLineOptions out = base;
            if (std::isfinite(options.celestial_line_max_time_step_s) &&
                options.celestial_line_max_time_step_s > 0.0)
            {
                out.max_time_step_s = options.celestial_line_max_time_step_s;
            }

            if (options.celestial_line_max_vertices_per_track > 1u)
            {
                out.max_vertices_per_arc =
                        std::min(out.max_vertices_per_arc,
                                 options.celestial_line_max_vertices_per_track);
                out.max_vertices_total =
                        std::min(out.max_vertices_total,
                                 options.celestial_line_max_vertices_per_track);
            }
            return out;
        }

        // Chooses the strongest nearby primary for a celestial track.
        orbitsim::BodyId select_celestial_kepler_primary_body_id(
                const orbitsim::GameSimulation &simulation,
                const KeplerPredictionSubject &subject,
                const KeplerWorldFrame &world_frame)
        {
            if (subject.body_id == orbitsim::kInvalidBodyId)
            {
                return orbitsim::kInvalidBodyId;
            }

            const orbitsim::MassiveBody *subject_body = simulation.body_by_id(subject.body_id);
            if (!subject_body || !finite_vec3(subject_body->state.position_m))
            {
                return world_frame.world_reference_body_id != subject.body_id
                               ? world_frame.world_reference_body_id
                               : orbitsim::kInvalidBodyId;
            }

            const double softening_m = simulation.config().softening_length_m;
            const double eps2 = (std::isfinite(softening_m) && softening_m > 0.0)
                                        ? softening_m * softening_m
                                        : 0.0;
            const orbitsim::MassiveBody *best_body = nullptr;
            double best_metric = -1.0;
            for (const orbitsim::MassiveBody &candidate : simulation.massive_bodies())
            {
                if (candidate.id == subject.body_id ||
                    !(candidate.mass_kg > 0.0) ||
                    !std::isfinite(candidate.mass_kg) ||
                    !finite_vec3(candidate.state.position_m))
                {
                    continue;
                }

                const orbitsim::Vec3 dr =
                        candidate.state.position_m - subject_body->state.position_m;
                const double r2 = dr.x * dr.x + dr.y * dr.y + dr.z * dr.z + eps2;
                if (!(r2 > 0.0) || !std::isfinite(r2))
                {
                    continue;
                }

                const double metric = candidate.mass_kg / r2;
                if (metric > best_metric)
                {
                    best_metric = metric;
                    best_body = &candidate;
                }
            }

            if (best_body)
            {
                return best_body->id;
            }
            return world_frame.world_reference_body_id != subject.body_id
                           ? world_frame.world_reference_body_id
                           : orbitsim::kInvalidBodyId;
        }

        // Builds one analytic Kepler track.
        KeplerPredictionState::Track build_kepler_track(
                const KeplerPredictionSubject &subject,
                const KeplerPredictionUpdateContext &context,
                const KeplerWorldFrame &world_frame,
                const KeplerBodyStateProvider &body_state_provider,
                const KeplerSharedCelestialEphemeris &ephemeris,
                const orbitsim::BodyId previous_primary_body_id)
        {
            if (!context.orbit || !context.orbit->scenario())
            {
                return make_invalid_track(subject, KeplerOrbitStatus::InvalidSimulation);
            }

            const orbitsim::GameSimulation &simulation = context.orbit->scenario()->sim;
            KeplerPredictionBuildRequest request{};
            request.simulation = &simulation;
            request.ephemeris = ephemeris.get();
            request.subject_state_inertial = subject.state_inertial;
            request.world_frame = world_frame;
            request.body_state_provider = body_state_provider;
            request.t0_s = context.current_sim_time_s;
            request.requested_horizon_s = context.requested_horizon_s;
            request.fixed_primary_body_id =
                    subject.celestial
                            ? select_celestial_kepler_primary_body_id(simulation, subject, world_frame)
                            : context.fixed_primary_body_id;
            request.current_primary_body_id =
                    subject.active_player ? previous_primary_body_id : orbitsim::kInvalidBodyId;
            request.options = context.options;
            if (subject.celestial)
            {
                request.options.patched_conics.enabled = false;
            }
            request.line_options =
                    subject.celestial
                            ? make_celestial_track_line_options(context.line_options, context.options)
                            : context.line_options;
            if (subject.active_player)
            {
                request.maneuver_nodes = context.maneuver_nodes;
                request.maneuver_revision = context.maneuver_revision;
            }

            KeplerPredictionBuildOutput built = build_kepler_prediction(request);

            KeplerPredictionState::Track track{};
            track.valid = built.valid;
            track.celestial = subject.celestial;
            track.active_player = subject.active_player;
            track.entity = subject.entity;
            track.body_id = subject.body_id;
            track.label = subject.label;
            track.orbit_rgb = subject.orbit_rgb;
            track.status = built.status;
            track.horizon_s = built.valid ? built.orbit.horizon_s : 0.0;
            track.primary_body_id = built.valid ? built.orbit.primary.body_id : orbitsim::kInvalidBodyId;
            track.orbit = std::move(built.orbit);
            track.world_frame = world_frame;
            track.body_state_provider = body_state_provider;
            track.line_options = request.line_options;
            track.line_propagation = request.line_options.propagation;
            track.base_arcs = std::move(built.base_arcs);
            track.planned_arcs = std::move(built.planned_arcs);
            track.base_patch_events = std::move(built.base_patch_events);
            track.planned_patch_events = std::move(built.planned_patch_events);
            track.first_planned_draw_arc_index = built.first_planned_draw_arc_index;
            track.planned_requested = built.planned_requested;
            track.planned_valid = built.planned_valid;
            track.planned_status = built.planned_status;
            track.planned_diagnostics = built.planned_diagnostics;
            track.metrics = built.metrics;
            track.perf = built.perf;
            return track;
        }

        // Builds celestial ground-truth lines from n-body ephemeris.
        KeplerPredictionState::Track build_celestial_nbody_track(
                const KeplerPredictionSubject &subject,
                const KeplerPredictionUpdateContext &context,
                const KeplerWorldFrame &world_frame,
                const KeplerSharedCelestialEphemeris &ephemeris,
                const double horizon_s)
        {
            if (!context.orbit ||
                !context.orbit->scenario() ||
                !ephemeris ||
                ephemeris->empty() ||
                !(horizon_s > 0.0) ||
                !std::isfinite(horizon_s))
            {
                return make_invalid_celestial_nbody_track(subject,
                                                          KeplerOrbitStatus::EphemerisUnavailable);
            }

            KeplerArcLineSet lines = build_kepler_celestial_nbody_lines(
                    KeplerCelestialNBodyLineRequest{
                            .ephemeris = ephemeris,
                            .body_id = subject.body_id,
                            .world_frame = world_frame,
                            .t0_s = context.current_sim_time_s,
                            .requested_horizon_s = horizon_s,
                            .line_options = make_celestial_track_line_options(context.line_options,
                                                                              context.options),
                    });

            KeplerPredictionState::Track track{};
            track.valid = lines.valid;
            track.celestial = true;
            track.celestial_nbody = true;
            track.active_player = subject.active_player;
            track.entity = subject.entity;
            track.body_id = subject.body_id;
            track.label = subject.label;
            track.orbit_rgb = subject.orbit_rgb;
            track.status = lines.valid ? KeplerOrbitStatus::Ok : lines.diagnostics.status;
            track.horizon_s = lines.valid ? horizon_s : 0.0;
            track.primary_body_id = world_frame.world_reference_body_id;
            track.prebuilt_base_lines = std::move(lines);
            return track;
        }

        double positive_or_default(const double value, const double fallback)
        {
            return (std::isfinite(value) && value > 0.0) ? value : fallback;
        }

        // Fallback when no subject can estimate the horizon.
        double fallback_prediction_horizon_s(const KeplerPredictionUpdateContext &context)
        {
            if (std::isfinite(context.requested_horizon_s) && context.requested_horizon_s > 0.0)
            {
                return context.requested_horizon_s;
            }
            return positive_or_default(context.options.open_orbit_window_s,
                                       24.0 * 60.0 * 60.0);
        }

        bool simulation_has_patched_conics_transition_candidates(
                const orbitsim::GameSimulation &simulation)
        {
            std::size_t massive_body_count = 0u;
            bool has_configured_soi = false;
            for (const orbitsim::MassiveBody &body : simulation.massive_bodies())
            {
                if (!(body.mass_kg > 0.0) || !std::isfinite(body.mass_kg))
                {
                    continue;
                }
                ++massive_body_count;
                has_configured_soi = has_configured_soi ||
                                     (body.soi_radius_m > 0.0 &&
                                      std::isfinite(body.soi_radius_m));
            }
            return massive_body_count > 1u && has_configured_soi;
        }

        bool has_patched_conics_subject(const std::span<const KeplerPredictionSubject> subjects)
        {
            return std::any_of(subjects.begin(),
                               subjects.end(),
                               [](const KeplerPredictionSubject &subject) {
                                   return !subject.celestial;
                               });
        }

        struct ResolvedCelestialNBodyHorizon
        {
            double uncapped_horizon_s{0.0};
            double horizon_s{0.0};
            double cap_s{0.0};
            bool capped{false};
        };

        struct EstimatedTrackHorizon
        {
            double base_horizon_s{0.0};
            double planned_preview_horizon_s{0.0};
        };

        // Estimates one subject's required display horizon.
        EstimatedTrackHorizon estimate_track_horizon_s(
                const KeplerPredictionSubject &subject,
                const KeplerPredictionUpdateContext &context,
                const KeplerWorldFrame &world_frame,
                const orbitsim::BodyId previous_primary_body_id)
        {
            EstimatedTrackHorizon out{};
            if (!context.orbit || !context.orbit->scenario())
            {
                return out;
            }

            const orbitsim::GameSimulation &simulation = context.orbit->scenario()->sim;
            KeplerBaseArcBuildRequest request{};
            request.simulation = &simulation;
            request.ephemeris = nullptr;
            request.subject_state_inertial = subject.state_inertial;
            request.t0_s = context.current_sim_time_s;
            request.requested_horizon_s = context.requested_horizon_s;
            request.fixed_primary_body_id =
                    subject.celestial
                            ? select_celestial_kepler_primary_body_id(simulation, subject, world_frame)
                            : context.fixed_primary_body_id;
            request.current_primary_body_id =
                    subject.active_player ? previous_primary_body_id : orbitsim::kInvalidBodyId;
            request.options = context.options;

            const KeplerBaseArcBuildResult orbit = build_kepler_base_arc(request);
            if (!orbit.valid || !std::isfinite(orbit.horizon_s) || !(orbit.horizon_s > 0.0))
            {
                return out;
            }

            out.base_horizon_s = orbit.horizon_s;
            if (subject.active_player && !context.maneuver_nodes.empty())
            {
                out.planned_preview_horizon_s =
                        required_kepler_planned_preview_horizon_s(
                                orbit,
                                context.maneuver_nodes.data(),
                                context.maneuver_nodes.size(),
                                context.options);
            }
            return out;
        }

        // Resolves the shared celestial ephemeris horizon.
        ResolvedCelestialNBodyHorizon resolve_required_celestial_nbody_horizon(
                const std::span<const KeplerPredictionSubject> subjects,
                const KeplerPredictionUpdateContext &context,
                const KeplerWorldFrame &world_frame,
                const orbitsim::BodyId previous_primary_body_id)
        {
            double uncapped_horizon_s = 0.0;
            double patched_conics_horizon_floor_s = 0.0;
            for (const KeplerPredictionSubject &subject : subjects)
            {
                const EstimatedTrackHorizon track_horizon =
                        estimate_track_horizon_s(subject,
                                                 context,
                                                 world_frame,
                                                 previous_primary_body_id);
                uncapped_horizon_s = std::max(uncapped_horizon_s,
                                              track_horizon.base_horizon_s);
                uncapped_horizon_s = std::max(uncapped_horizon_s,
                                              track_horizon.planned_preview_horizon_s);
                if (context.options.patched_conics.enabled && !subject.celestial)
                {
                    patched_conics_horizon_floor_s = std::max(patched_conics_horizon_floor_s,
                                                              track_horizon.base_horizon_s);
                    patched_conics_horizon_floor_s = std::max(patched_conics_horizon_floor_s,
                                                              track_horizon.planned_preview_horizon_s);
                }
            }
            const double maneuver_node_horizon_s =
                    required_kepler_maneuver_node_horizon_s(context.current_sim_time_s,
                                                             context.maneuver_nodes.data(),
                                                             context.maneuver_nodes.size());
            uncapped_horizon_s = std::max(uncapped_horizon_s,
                                          maneuver_node_horizon_s);

            if (!(uncapped_horizon_s > 0.0) || !std::isfinite(uncapped_horizon_s))
            {
                const KeplerCelestialNBodyHorizonLimit fallback =
                        limit_kepler_celestial_nbody_horizon(
                                fallback_prediction_horizon_s(context),
                                context.options,
                                patched_conics_horizon_floor_s);
                return ResolvedCelestialNBodyHorizon{
                        .uncapped_horizon_s = fallback.uncapped_horizon_s,
                        .horizon_s = fallback.horizon_s,
                        .cap_s = fallback.cap_s,
                        .capped = fallback.capped,
                };
            }

            const KeplerCelestialNBodyHorizonLimit limited =
                    limit_kepler_celestial_nbody_horizon(uncapped_horizon_s,
                                                         context.options,
                                                         patched_conics_horizon_floor_s);
            return ResolvedCelestialNBodyHorizon{
                    .uncapped_horizon_s = uncapped_horizon_s,
                    .horizon_s = limited.horizon_s,
                    .cap_s = limited.cap_s,
                    .capped = limited.capped,
            };
        }

        // Throttles whole-track rebuilds. Patched conics rebuilds are relatively
        // expensive because they run SOI transition search plus line resampling.
        double prediction_rebuild_interval_s(const KeplerPredictionUpdateContext &context)
        {
            const double source_dt_s =
                    (std::isfinite(context.line_options.max_time_step_s) &&
                     context.line_options.max_time_step_s > 0.0)
                            ? context.line_options.max_time_step_s
                            : 10.0;

            if (context.options.patched_conics.enabled)
            {
                return 0.05;
            }

            if (!context.maneuver_nodes.empty())
            {
                return std::clamp(source_dt_s, 1.0, 60.0);
            }

            return std::clamp(source_dt_s * 0.005, 1.0 / 60.0, 0.05);
        }

        double prediction_rebuild_wall_interval_s(const KeplerPredictionUpdateContext &context)
        {
            if (!context.options.patched_conics.enabled)
            {
                return 0.0;
            }

            return !context.maneuver_nodes.empty() ? 0.20 : 0.10;
        }

        // Checks whether the current prediction can be reused.
        bool can_reuse_prediction(const KeplerPredictionState &state,
                                  const KeplerPredictionUpdateContext &context,
                                  const KeplerWorldFrame &world_frame,
                                  const KeplerPredictionInputFingerprint &input_fingerprint)
        {
            if (state.dirty || !state.valid)
            {
                return false;
            }
            if (state.maneuver_revision != context.maneuver_revision ||
                state.world_reference_body_id != world_frame.world_reference_body_id)
            {
                return false;
            }
            if (!(state.input_fingerprint == input_fingerprint))
            {
                return false;
            }
            if (!std::isfinite(state.build_time_s) ||
                !std::isfinite(context.current_sim_time_s) ||
                context.current_sim_time_s < state.build_time_s)
            {
                return false;
            }

            const double elapsed_s = context.current_sim_time_s - state.build_time_s;
            const double sim_interval_s = prediction_rebuild_interval_s(context);
            const double wall_interval_s = prediction_rebuild_wall_interval_s(context);
            if (wall_interval_s > 0.0 &&
                std::isfinite(state.build_wall_time_s) &&
                std::isfinite(context.current_wall_time_s) &&
                context.current_wall_time_s >= state.build_wall_time_s)
            {
                const double wall_elapsed_s = context.current_wall_time_s - state.build_wall_time_s;
                if (wall_elapsed_s < wall_interval_s &&
                    elapsed_s < sim_interval_s)
                {
                    return true;
                }
            }

            if (elapsed_s < sim_interval_s)
            {
                return true;
            }

            if (!context.maneuver_nodes.empty() &&
                std::isfinite(state.build_wall_time_s) &&
                std::isfinite(context.current_wall_time_s) &&
                context.current_wall_time_s >= state.build_wall_time_s)
            {
                constexpr double kPlannedWallRebuildIntervalS = 0.20;
                return (context.current_wall_time_s - state.build_wall_time_s) <
                               kPlannedWallRebuildIntervalS &&
                       elapsed_s < sim_interval_s;
            }

            return false;
        }
    } // namespace

    // Reuses or rebuilds the shared celestial ephemeris.
    const KeplerPredictionSystem::CelestialNBodyEphemerisCache &
    KeplerPredictionSystem::resolve_celestial_nbody_cache(
            const KeplerPredictionUpdateContext &context,
            const KeplerWorldFrame &world_frame,
            const double required_horizon_s,
            const double uncapped_required_horizon_s,
            const double horizon_cap_s,
            const bool horizon_capped)
    {
        if (!context.orbit || !context.orbit->scenario() || !std::isfinite(context.current_sim_time_s))
        {
            _celestial_nbody_cache = {};
            _celestial_nbody_cache.status = KeplerOrbitStatus::InvalidSimulation;
            return _celestial_nbody_cache;
        }

        const orbitsim::GameSimulation &simulation = context.orbit->scenario()->sim;
        KeplerCelestialNBodyEphemerisRequest horizon_request{};
        horizon_request.simulation = &simulation;
        horizon_request.world_frame = world_frame;
        horizon_request.t0_s = context.current_sim_time_s;
        horizon_request.requested_horizon_s = required_horizon_s;
        horizon_request.options = context.options;

        const double required_end_s = context.current_sim_time_s + required_horizon_s;
        if (!(required_horizon_s > 0.0) ||
            !std::isfinite(required_horizon_s) ||
            !std::isfinite(required_end_s))
        {
            _celestial_nbody_cache = {};
            _celestial_nbody_cache.status = KeplerOrbitStatus::InvalidInput;
            return _celestial_nbody_cache;
        }

        bool cache_matches = _celestial_nbody_cache.valid &&
                             _celestial_nbody_cache.simulation == &simulation &&
                             _celestial_nbody_cache.ephemeris &&
                             !_celestial_nbody_cache.ephemeris->empty() &&
                             _celestial_nbody_cache.world_reference_body_id ==
                                     world_frame.world_reference_body_id &&
                             _celestial_nbody_cache.gravitational_constant ==
                                     simulation.config().gravitational_constant &&
                             _celestial_nbody_cache.softening_length_m ==
                                     simulation.config().softening_length_m &&
                             _celestial_nbody_cache.ephemeris_options ==
                                     context.options.celestial_nbody_ephemeris;

        const std::vector<orbitsim::MassiveBody> &bodies = simulation.massive_bodies();
        cache_matches = cache_matches &&
                        _celestial_nbody_cache.body_ids.size() == bodies.size() &&
                        _celestial_nbody_cache.body_masses_kg.size() == bodies.size();
        if (cache_matches)
        {
            for (std::size_t i = 0; i < bodies.size(); ++i)
            {
                if (_celestial_nbody_cache.body_ids[i] != bodies[i].id ||
                    _celestial_nbody_cache.body_masses_kg[i] != bodies[i].mass_kg)
                {
                    cache_matches = false;
                    break;
                }
            }
        }

        constexpr double kTimeEpsilonS = 1.0e-6;
        const bool cache_covers = _celestial_nbody_cache.t0_s <= context.current_sim_time_s + kTimeEpsilonS &&
                                  _celestial_nbody_cache.t_end_s + kTimeEpsilonS >= required_end_s;
        if (cache_matches && cache_covers)
        {
            _celestial_nbody_cache.uncapped_required_horizon_s =
                    uncapped_required_horizon_s;
            _celestial_nbody_cache.required_horizon_s = required_horizon_s;
            _celestial_nbody_cache.horizon_cap_s = horizon_cap_s;
            _celestial_nbody_cache.horizon_capped = horizon_capped;
            return _celestial_nbody_cache;
        }

        KeplerCelestialNBodyEphemerisRequest build_request = horizon_request;
        double build_horizon_cap_s = 0.0;
        if (std::isfinite(horizon_cap_s) && horizon_cap_s > 0.0)
        {
            build_horizon_cap_s = std::max({
                    required_horizon_s,
                    horizon_cap_s,
                    required_horizon_s + kepler_prediction_cache_reuse_horizon_s(required_horizon_s),
            });
        }
        const double build_horizon_s =
                kepler_prediction_build_horizon_s(required_horizon_s, build_horizon_cap_s);
        build_request.requested_horizon_s = build_horizon_s;

        const KeplerCelestialNBodyEphemerisResult built =
                build_kepler_celestial_nbody_ephemeris(build_request);

        CelestialNBodyEphemerisCache next_cache{};
        next_cache.status = built.status;
        next_cache.diagnostics = built.diagnostics;
        next_cache.uncapped_required_horizon_s = uncapped_required_horizon_s;
        next_cache.required_horizon_s = required_horizon_s;
        next_cache.built_horizon_s = built.horizon_s;
        next_cache.horizon_cap_s = horizon_cap_s;
        next_cache.horizon_capped = horizon_capped;

        if (!built.valid || !built.ephemeris || built.ephemeris->empty())
        {
            _celestial_nbody_cache = std::move(next_cache);
            return _celestial_nbody_cache;
        }

        next_cache.valid = true;
        next_cache.simulation = &simulation;
        next_cache.ephemeris = built.ephemeris;
        next_cache.body_state_provider = built.body_state_provider;
        next_cache.t0_s = built.ephemeris->t0_s();
        next_cache.t_end_s = built.ephemeris->t_end_s();
        next_cache.gravitational_constant = simulation.config().gravitational_constant;
        next_cache.softening_length_m = simulation.config().softening_length_m;
        next_cache.ephemeris_options = context.options.celestial_nbody_ephemeris;
        next_cache.world_reference_body_id = world_frame.world_reference_body_id;
        next_cache.body_ids.reserve(bodies.size());
        next_cache.body_masses_kg.reserve(bodies.size());
        for (const orbitsim::MassiveBody &body : bodies)
        {
            next_cache.body_ids.push_back(body.id);
            next_cache.body_masses_kg.push_back(body.mass_kg);
        }

        _celestial_nbody_cache = std::move(next_cache);
        return _celestial_nbody_cache;
    }

    void KeplerPredictionSystem::reset()
    {
        _state = {};
        _celestial_nbody_cache = {};
    }

    void KeplerPredictionSystem::mark_dirty()
    {
        _state.dirty = true;
    }

    // Main update pipeline for all Kepler prediction tracks.
    void KeplerPredictionSystem::update(const KeplerPredictionUpdateContext &context)
    {
        const auto update_start = KeplerPerfClock::now();
        _state.enabled = context.enabled;
        if (!context.enabled)
        {
            _state.clear_result();
            _state.dirty = false;
            return;
        }

        if (!context.orbit || !context.world || !context.scenario_config ||
            !context.orbit->scenario())
        {
            _state.clear_result(KeplerOrbitStatus::InvalidSimulation);
            _state.dirty = false;
            return;
        }

        KeplerWorldFrame world_frame{};
        if (!resolve_kepler_prediction_world_frame(KeplerPredictionSubjectContext{
                    .orbit = context.orbit,
                    .world = context.world,
                    .physics = context.physics,
                    .physics_context = context.physics_context,
                    .scenario_config = context.scenario_config,
                },
                                                   world_frame))
        {
            _state.clear_result(KeplerOrbitStatus::InvalidSimulation);
            _state.dirty = false;
            return;
        }

        const KeplerPredictionInputFingerprint input_fingerprint =
                make_kepler_prediction_input_fingerprint(context, world_frame);
        if (can_reuse_prediction(_state, context, world_frame, input_fingerprint))
        {
            // Existing tracks still cover this frame.
            _state.enabled = context.enabled;
            _state.perf.reused_last_update = true;
            _state.perf.rebuilt_last_update = false;
            _state.perf.last_update_ms = elapsed_ms(update_start);
            return;
        }

        KeplerPredictionState::PerfDebug perf{};
        perf.rebuilt_last_update = true;
        perf.rebuild_count = _state.perf.rebuild_count + 1u;

        const KeplerPredictionSubjectContext subject_context{
                .orbit = context.orbit,
                .world = context.world,
                .physics = context.physics,
                .physics_context = context.physics_context,
                .scenario_config = context.scenario_config,
        };

        const auto subject_start = KeplerPerfClock::now();
        std::vector<KeplerPredictionSubject> subjects =
                resolve_kepler_prediction_orbiter_subjects(subject_context, world_frame);
        const bool needs_celestial_subjects =
                context.build_celestial_kepler_tracks || context.build_celestial_nbody_tracks;
        std::vector<KeplerPredictionSubject> celestial_subjects =
                needs_celestial_subjects
                        ? resolve_kepler_prediction_celestial_subjects(subject_context)
                        : std::vector<KeplerPredictionSubject>{};
        const std::size_t celestial_subject_count = celestial_subjects.size();
        if (context.build_celestial_kepler_tracks)
        {
            subjects.insert(subjects.end(),
                            celestial_subjects.begin(),
                            celestial_subjects.end());
        }
        perf.subject_resolve_ms = elapsed_ms(subject_start);
        perf.subject_count = subjects.size();
        perf.celestial_subject_count = celestial_subject_count;
        if (subjects.empty() && (!context.build_celestial_nbody_tracks || celestial_subjects.empty()))
        {
            _state.clear_result(KeplerOrbitStatus::InvalidSubjectState);
            _state.dirty = false;
            return;
        }

        const orbitsim::BodyId previous_primary_body_id = _state.primary_body_id;
        ++_state.revision;
        _state.build_time_s = context.current_sim_time_s;
        _state.build_wall_time_s = context.current_wall_time_s;
        _state.maneuver_revision = context.maneuver_revision;
        _state.input_fingerprint = input_fingerprint;
        _state.world_reference_body_id = world_frame.world_reference_body_id;
        _state.dirty = false;
        _state.tracks.clear();
        _state.tracks.reserve(subjects.size() +
                              (context.build_celestial_nbody_tracks ? celestial_subject_count : 0u));

        const bool patched_conics_needs_ephemeris =
                context.options.patched_conics.enabled &&
                has_patched_conics_subject(
                        std::span<const KeplerPredictionSubject>(subjects.data(), subjects.size())) &&
                simulation_has_patched_conics_transition_candidates(context.orbit->scenario()->sim);
        const bool needs_celestial_nbody_cache =
                context.build_celestial_nbody_tracks ||
                context.build_celestial_kepler_tracks ||
                patched_conics_needs_ephemeris;
        perf.patched_conics_needs_ephemeris = patched_conics_needs_ephemeris;
        perf.needs_celestial_nbody_cache = needs_celestial_nbody_cache;
        // Share one celestial ephemeris across all tracks.
        const auto horizon_start = KeplerPerfClock::now();
        ResolvedCelestialNBodyHorizon required_celestial_nbody_horizon =
                needs_celestial_nbody_cache
                        ? resolve_required_celestial_nbody_horizon(
                                  std::span<const KeplerPredictionSubject>(subjects.data(), subjects.size()),
                                  context,
                                  world_frame,
                                  previous_primary_body_id)
                        : ResolvedCelestialNBodyHorizon{};
        perf.horizon_resolve_ms = elapsed_ms(horizon_start);
        perf.required_ephemeris_horizon_s = required_celestial_nbody_horizon.horizon_s;
        perf.uncapped_ephemeris_horizon_s = required_celestial_nbody_horizon.uncapped_horizon_s;

        const auto publish_ephemeris_debug = [this, needs_celestial_nbody_cache](
                                                      const CelestialNBodyEphemerisCache &cache) {
            if (!needs_celestial_nbody_cache)
            {
                _state.celestial_nbody_ephemeris = {};
                return;
            }

            _state.celestial_nbody_ephemeris.valid = cache.valid;
            _state.celestial_nbody_ephemeris.status = cache.status;
            _state.celestial_nbody_ephemeris.uncapped_required_horizon_s =
                    cache.uncapped_required_horizon_s;
            _state.celestial_nbody_ephemeris.required_horizon_s =
                    cache.required_horizon_s;
            _state.celestial_nbody_ephemeris.built_horizon_s =
                    cache.built_horizon_s;
            _state.celestial_nbody_ephemeris.horizon_cap_s =
                    cache.horizon_cap_s;
            _state.celestial_nbody_ephemeris.t0_s = cache.t0_s;
            _state.celestial_nbody_ephemeris.t_end_s = cache.t_end_s;
            _state.celestial_nbody_ephemeris.body_count = cache.body_ids.size();
            _state.celestial_nbody_ephemeris.accepted_segments =
                    cache.diagnostics.accepted_segments;
            _state.celestial_nbody_ephemeris.rejected_splits =
                    cache.diagnostics.rejected_splits;
            _state.celestial_nbody_ephemeris.forced_boundary_splits =
                    cache.diagnostics.forced_boundary_splits;
            _state.celestial_nbody_ephemeris.min_dt_s =
                    cache.diagnostics.min_dt_s;
            _state.celestial_nbody_ephemeris.avg_dt_s =
                    cache.diagnostics.avg_dt_s;
            _state.celestial_nbody_ephemeris.max_dt_s =
                    cache.diagnostics.max_dt_s;
            _state.celestial_nbody_ephemeris.horizon_capped =
                    cache.horizon_capped;
            _state.celestial_nbody_ephemeris.hard_cap_hit =
                    cache.diagnostics.hard_cap_hit;
            _state.celestial_nbody_ephemeris.cancelled =
                    cache.diagnostics.cancelled;
        };

        const auto build_tracks = [&](const CelestialNBodyEphemerisCache &cache,
                                      const double celestial_nbody_horizon_s) {
            _state.tracks.clear();
            _state.tracks.reserve(subjects.size() +
                                  (context.build_celestial_nbody_tracks ? celestial_subject_count : 0u));

            const KeplerBodyStateProvider body_state_provider =
                    needs_celestial_nbody_cache && cache.valid
                            ? cache.body_state_provider
                            : make_current_kepler_body_state_provider(*context.orbit);

            // Analytic Kepler tracks.
            for (const KeplerPredictionSubject &subject : subjects)
            {
                _state.tracks.push_back(build_kepler_track(subject,
                                                           context,
                                                           world_frame,
                                                           body_state_provider,
                                                           cache.ephemeris,
                                                           previous_primary_body_id));
            }

            if (context.build_celestial_nbody_tracks)
            {
                // Sampled n-body celestial tracks.
                for (const KeplerPredictionSubject &subject : celestial_subjects)
                {
                    _state.tracks.push_back(build_celestial_nbody_track(subject,
                                                                        context,
                                                                        world_frame,
                                                                        cache.ephemeris,
                                                                        celestial_nbody_horizon_s));
                }
            }
        };

        const auto planned_ephemeris_horizon_s = [&](const double ephemeris_end_s) {
            double horizon_s = 0.0;
            for (const KeplerPredictionState::Track &track : _state.tracks)
            {
                if (!track.active_player)
                {
                    continue;
                }
                horizon_s = std::max(
                        horizon_s,
                        required_kepler_planned_preview_ephemeris_horizon_s(
                                std::span<const KeplerOrbitArc>(track.planned_arcs.data(),
                                                                track.planned_arcs.size()),
                                std::span<const KeplerPatchEvent>(track.planned_patch_events.data(),
                                                                  track.planned_patch_events.size()),
                                context.current_sim_time_s,
                                ephemeris_end_s,
                                context.options));
            }
            return horizon_s;
        };

        const CelestialNBodyEphemerisCache *celestial_nbody =
                needs_celestial_nbody_cache
                        ? [&]() {
                              const auto ephemeris_start = KeplerPerfClock::now();
                              const CelestialNBodyEphemerisCache &cache =
                                      resolve_celestial_nbody_cache(
                                              context,
                                              world_frame,
                                              required_celestial_nbody_horizon.horizon_s,
                                              required_celestial_nbody_horizon.uncapped_horizon_s,
                                              required_celestial_nbody_horizon.cap_s,
                                              required_celestial_nbody_horizon.capped);
                              perf.ephemeris_resolve_ms = elapsed_ms(ephemeris_start);
                              return &cache;
                          }()
                        : &_celestial_nbody_cache;

        const auto build_tracks_start = KeplerPerfClock::now();
        build_tracks(*celestial_nbody, required_celestial_nbody_horizon.horizon_s);
        perf.build_tracks_ms = elapsed_ms(build_tracks_start);

        if (patched_conics_needs_ephemeris &&
            celestial_nbody->valid &&
            std::isfinite(celestial_nbody->t_end_s))
        {
            const double planned_horizon_s =
                    planned_ephemeris_horizon_s(celestial_nbody->t_end_s);
            perf.planned_ephemeris_horizon_s = planned_horizon_s;
            if (planned_horizon_s > celestial_nbody->required_horizon_s + 1.0e-6)
            {
                perf.second_ephemeris_pass = true;
                const double uncapped_horizon_s =
                        std::max(required_celestial_nbody_horizon.uncapped_horizon_s,
                                 planned_horizon_s);
                const KeplerCelestialNBodyHorizonLimit limited =
                        limit_kepler_celestial_nbody_horizon(uncapped_horizon_s,
                                                             context.options,
                                                             planned_horizon_s);
                required_celestial_nbody_horizon = ResolvedCelestialNBodyHorizon{
                        .uncapped_horizon_s = uncapped_horizon_s,
                        .horizon_s = limited.horizon_s,
                        .cap_s = limited.cap_s,
                        .capped = limited.capped,
                };
                perf.required_ephemeris_horizon_s = required_celestial_nbody_horizon.horizon_s;
                perf.uncapped_ephemeris_horizon_s = required_celestial_nbody_horizon.uncapped_horizon_s;
                const auto second_ephemeris_start = KeplerPerfClock::now();
                celestial_nbody =
                        &resolve_celestial_nbody_cache(context,
                                                       world_frame,
                                                       required_celestial_nbody_horizon.horizon_s,
                                                       required_celestial_nbody_horizon.uncapped_horizon_s,
                                                       required_celestial_nbody_horizon.cap_s,
                                                       required_celestial_nbody_horizon.capped);
                perf.second_ephemeris_resolve_ms = elapsed_ms(second_ephemeris_start);
                const auto second_build_tracks_start = KeplerPerfClock::now();
                build_tracks(*celestial_nbody, required_celestial_nbody_horizon.horizon_s);
                perf.second_build_tracks_ms = elapsed_ms(second_build_tracks_start);
            }
        }

        publish_ephemeris_debug(*celestial_nbody);
        publish_representative_track(_state);
        record_perf_tracks(perf, _state.tracks);
        perf.last_update_ms = elapsed_ms(update_start);
        perf.last_rebuild_ms = perf.last_update_ms;
        _state.perf = perf;
    }

} // namespace Game
