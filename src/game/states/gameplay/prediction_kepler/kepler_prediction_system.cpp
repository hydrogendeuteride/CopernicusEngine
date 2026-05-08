#include "game/states/gameplay/prediction_kepler/kepler_prediction_system.h"

#include "game/orbit/kepler/kepler_celestial_nbody.h"
#include "game/states/gameplay/orbital_runtime_system.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <utility>
#include <vector>

namespace Game
{
    namespace
    {
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
            state.orbit = representative->orbit;
            state.base_arcs = representative->base_arcs;
            state.planned_arcs = representative->planned_arcs;
            state.base_lines = representative->base_lines;
            state.planned_lines = representative->planned_lines;
            state.metrics = representative->metrics;
        }

        bool finite_vec3(const orbitsim::Vec3 &v)
        {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        }

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

        KeplerOrbitTessellationOptions make_celestial_track_tessellation(
                const KeplerOrbitTessellationOptions &base,
                const KeplerPredictionOptions &options)
        {
            KeplerOrbitTessellationOptions out = base;
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
            request.tessellation =
                    subject.celestial
                            ? make_celestial_track_tessellation(context.tessellation, context.options)
                            : context.tessellation;
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
            track.base_arcs = std::move(built.base_arcs);
            track.planned_arcs = std::move(built.planned_arcs);
            track.base_lines = std::move(built.base_lines);
            track.planned_lines = std::move(built.planned_lines);
            track.metrics = built.metrics;
            return track;
        }

        KeplerPredictionState::Track build_celestial_nbody_track(
                const KeplerPredictionSubject &subject,
                const KeplerPredictionUpdateContext &context,
                const KeplerWorldFrame &world_frame,
                const KeplerSharedCelestialEphemeris &ephemeris)
        {
            if (!context.orbit || !context.orbit->scenario() || !ephemeris || ephemeris->empty())
            {
                return make_invalid_celestial_nbody_track(subject,
                                                          KeplerOrbitStatus::EphemerisUnavailable);
            }

            const orbitsim::GameSimulation &simulation = context.orbit->scenario()->sim;
            const double horizon_s =
                    select_kepler_celestial_nbody_horizon_s(simulation,
                                                            subject.body_id,
                                                            context.options,
                                                            context.celestial_nbody_horizon_s);
            KeplerOrbitLineSet lines = build_kepler_celestial_nbody_lines(
                    KeplerCelestialNBodyLineRequest{
                            .ephemeris = ephemeris,
                            .body_id = subject.body_id,
                            .world_frame = world_frame,
                            .t0_s = context.current_sim_time_s,
                            .requested_horizon_s = horizon_s,
                            .tessellation = make_celestial_track_tessellation(context.tessellation,
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
            track.base_lines = std::move(lines);
            return track;
        }

        double positive_or_default(const double value, const double fallback)
        {
            return (std::isfinite(value) && value > 0.0) ? value : fallback;
        }

        double cache_reuse_horizon_s(const double required_horizon_s)
        {
            constexpr double kMinReuseS = 10.0 * 60.0;
            constexpr double kMaxReuseS = 7.0 * 24.0 * 60.0 * 60.0;
            if (!(required_horizon_s > 0.0) || !std::isfinite(required_horizon_s))
            {
                return kMinReuseS;
            }
            return std::clamp(required_horizon_s * 0.25, kMinReuseS, kMaxReuseS);
        }

        double prediction_rebuild_interval_s(const KeplerPredictionUpdateContext &context)
        {
            const double source_dt_s =
                    (std::isfinite(context.tessellation.max_time_step_s) &&
                     context.tessellation.max_time_step_s > 0.0)
                            ? context.tessellation.max_time_step_s
                            : 10.0;
            return std::clamp(source_dt_s * 0.005, 1.0 / 60.0, 0.05);
        }

        bool can_reuse_prediction(const KeplerPredictionState &state,
                                  const KeplerPredictionUpdateContext &context,
                                  const KeplerWorldFrame &world_frame)
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
            if (!std::isfinite(state.build_time_s) ||
                !std::isfinite(context.current_sim_time_s) ||
                context.current_sim_time_s < state.build_time_s)
            {
                return false;
            }

            const double elapsed_s = context.current_sim_time_s - state.build_time_s;
            return elapsed_s < prediction_rebuild_interval_s(context);
        }
    } // namespace

    const KeplerPredictionSystem::CelestialNBodyEphemerisCache &
    KeplerPredictionSystem::resolve_celestial_nbody_cache(
            const KeplerPredictionUpdateContext &context,
            const KeplerWorldFrame &world_frame)
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
        horizon_request.requested_horizon_s = context.celestial_nbody_horizon_s;
        horizon_request.options = context.options;

        const double required_horizon_s =
                select_kepler_celestial_nbody_ephemeris_horizon_s(horizon_request);
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
            _celestial_nbody_cache.required_horizon_s = required_horizon_s;
            return _celestial_nbody_cache;
        }

        KeplerCelestialNBodyEphemerisRequest build_request = horizon_request;
        const double build_horizon_s = required_horizon_s + cache_reuse_horizon_s(required_horizon_s);
        build_request.requested_horizon_s = build_horizon_s;

        const KeplerCelestialNBodyEphemerisResult built =
                build_kepler_celestial_nbody_ephemeris(build_request);

        CelestialNBodyEphemerisCache next_cache{};
        next_cache.status = built.status;
        next_cache.diagnostics = built.diagnostics;
        next_cache.required_horizon_s = required_horizon_s;
        next_cache.built_horizon_s = built.horizon_s;

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

    void KeplerPredictionSystem::update(const KeplerPredictionUpdateContext &context)
    {
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

        if (can_reuse_prediction(_state, context, world_frame))
        {
            _state.enabled = context.enabled;
            return;
        }

        const KeplerPredictionSubjectContext subject_context{
                .orbit = context.orbit,
                .world = context.world,
                .physics = context.physics,
                .physics_context = context.physics_context,
                .scenario_config = context.scenario_config,
        };

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
        if (subjects.empty() && (!context.build_celestial_nbody_tracks || celestial_subjects.empty()))
        {
            _state.clear_result(KeplerOrbitStatus::InvalidSubjectState);
            _state.dirty = false;
            return;
        }

        const orbitsim::BodyId previous_primary_body_id = _state.primary_body_id;
        ++_state.revision;
        _state.build_time_s = context.current_sim_time_s;
        _state.maneuver_revision = context.maneuver_revision;
        _state.world_reference_body_id = world_frame.world_reference_body_id;
        _state.dirty = false;
        _state.tracks.clear();
        _state.tracks.reserve(subjects.size() +
                              (context.build_celestial_nbody_tracks ? celestial_subject_count : 0u));

        const bool needs_celestial_nbody_cache =
                context.build_celestial_nbody_tracks || context.build_celestial_kepler_tracks;
        const CelestialNBodyEphemerisCache &celestial_nbody =
                needs_celestial_nbody_cache
                        ? resolve_celestial_nbody_cache(context, world_frame)
                        : _celestial_nbody_cache;
        _state.celestial_nbody_ephemeris.valid = celestial_nbody.valid;
        _state.celestial_nbody_ephemeris.status = celestial_nbody.status;
        _state.celestial_nbody_ephemeris.required_horizon_s =
                celestial_nbody.required_horizon_s;
        _state.celestial_nbody_ephemeris.built_horizon_s =
                celestial_nbody.built_horizon_s;
        _state.celestial_nbody_ephemeris.t0_s = celestial_nbody.t0_s;
        _state.celestial_nbody_ephemeris.t_end_s = celestial_nbody.t_end_s;
        _state.celestial_nbody_ephemeris.body_count = celestial_nbody.body_ids.size();
        _state.celestial_nbody_ephemeris.accepted_segments =
                celestial_nbody.diagnostics.accepted_segments;
        _state.celestial_nbody_ephemeris.rejected_splits =
                celestial_nbody.diagnostics.rejected_splits;
        _state.celestial_nbody_ephemeris.forced_boundary_splits =
                celestial_nbody.diagnostics.forced_boundary_splits;
        _state.celestial_nbody_ephemeris.min_dt_s =
                celestial_nbody.diagnostics.min_dt_s;
        _state.celestial_nbody_ephemeris.avg_dt_s =
                celestial_nbody.diagnostics.avg_dt_s;
        _state.celestial_nbody_ephemeris.max_dt_s =
                celestial_nbody.diagnostics.max_dt_s;
        _state.celestial_nbody_ephemeris.hard_cap_hit =
                celestial_nbody.diagnostics.hard_cap_hit;
        _state.celestial_nbody_ephemeris.cancelled =
                celestial_nbody.diagnostics.cancelled;
        if (!needs_celestial_nbody_cache)
        {
            _state.celestial_nbody_ephemeris = {};
        }

        const KeplerBodyStateProvider body_state_provider =
                needs_celestial_nbody_cache && celestial_nbody.valid
                        ? celestial_nbody.body_state_provider
                        : make_current_kepler_body_state_provider(*context.orbit);

        for (const KeplerPredictionSubject &subject : subjects)
        {
            _state.tracks.push_back(build_kepler_track(subject,
                                                       context,
                                                       world_frame,
                                                       body_state_provider,
                                                       celestial_nbody.ephemeris,
                                                       previous_primary_body_id));
        }

        if (context.build_celestial_nbody_tracks)
        {
            for (const KeplerPredictionSubject &subject : celestial_subjects)
            {
                _state.tracks.push_back(build_celestial_nbody_track(subject,
                                                                    context,
                                                                    world_frame,
                                                                    celestial_nbody.ephemeris));
            }
        }

        publish_representative_track(_state);
    }

} // namespace Game
