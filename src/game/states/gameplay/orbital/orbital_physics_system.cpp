#include "game/states/gameplay/orbital/orbital_physics_system.h"

#include "core/game_api.h"
#include "core/input/input_system.h"
#include "game/component/ship_controller.h"
#include "game/game_world.h"
#include "game/states/gameplay/orbital/orbit_runtime.h"
#include "game/states/gameplay/orbital/orbital_runtime_system.h"
#include "game/states/gameplay/orbital/orbiter_state_bridge.h"
#include "game/states/gameplay/scenario/scenario_config.h"
#include "game/state/game_state.h"
#include "orbitsim/detail/simulation_stepping.hpp"
#include "orbitsim/kepler_trajectory.hpp"
#include "orbitsim/soi.hpp"
#include "physics/physics_context.h"
#include "physics/physics_world.h"

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace Game
{
    using detail::finite_vec3;
    using detail::nbody_accel_body_centered;
    using detail::select_soi_primary_body;
    using detail::soi_accel_body_centered;

    namespace
    {
        constexpr double kRuntimePatchTimeEpsilonS = 1.0e-9;
        constexpr double kRuntimePatchRefineToleranceS = 0.25;
        constexpr double kRuntimeTransitionSearchSamplesPerStep = 4.0;
        constexpr std::size_t kRuntimeMaxPatchesPerStep = 8u;

        bool finite_state(const orbitsim::State &state)
        {
            return finite_vec3(state.position_m) &&
                   finite_vec3(state.velocity_mps) &&
                   finite_vec3(state.spin.axis) &&
                   std::isfinite(state.spin.angle_rad) &&
                   std::isfinite(state.spin.rate_rad_per_s);
        }

        bool runtime_has_other_soi_candidate(const orbitsim::GameSimulation &sim,
                                             const orbitsim::BodyId current_primary_body_id)
        {
            for (const orbitsim::MassiveBody &body : sim.massive_bodies())
            {
                if (body.id != current_primary_body_id &&
                    body.soi_radius_m > 0.0 &&
                    std::isfinite(body.soi_radius_m))
                {
                    return true;
                }
            }
            return false;
        }

        double runtime_transition_search_step_s(const OrbitalPhysicsSystem::Context &context,
                                                const double t0_s,
                                                const double t1_s)
        {
            const double fallback_step_s =
                    std::isfinite(context.soi_kepler_max_step_s) && context.soi_kepler_max_step_s > 0.0
                            ? context.soi_kepler_max_step_s
                            : 60.0;
            const double span_s = std::abs(t1_s - t0_s);
            if (!(span_s > 0.0) || !std::isfinite(span_s))
            {
                return fallback_step_s;
            }

            return std::min(fallback_step_s, span_s / kRuntimeTransitionSearchSamplesPerStep);
        }

        orbitsim::CelestialEphemeris make_runtime_step_ephemeris(
                const std::vector<orbitsim::BodyId> &body_ids,
                const std::vector<orbitsim::State> &start_states,
                const std::vector<orbitsim::State> &end_states,
                const double t0_s,
                const double t1_s)
        {
            orbitsim::CelestialEphemeris ephemeris{};
            if (body_ids.empty() ||
                body_ids.size() != start_states.size() ||
                body_ids.size() != end_states.size() ||
                !std::isfinite(t0_s) ||
                !std::isfinite(t1_s))
            {
                return ephemeris;
            }

            ephemeris.set_body_ids(body_ids);
            orbitsim::CelestialEphemerisSegment segment{};
            segment.t0_s = t0_s;
            segment.dt_s = t1_s - t0_s;
            segment.start = start_states;
            segment.end = end_states;
            ephemeris.segments.push_back(std::move(segment));
            return ephemeris;
        }

        void capture_runtime_body_start_states(const orbitsim::GameSimulation &sim,
                                               std::vector<orbitsim::BodyId> &body_ids,
                                               std::vector<orbitsim::State> &body_states)
        {
            body_ids.clear();
            body_states.clear();
            body_ids.reserve(sim.massive_bodies().size());
            body_states.reserve(sim.massive_bodies().size());

            for (const orbitsim::MassiveBody &body : sim.massive_bodies())
            {
                if (finite_state(body.state))
                {
                    body_ids.push_back(body.id);
                    body_states.push_back(body.state);
                }
            }
        }

        bool capture_runtime_body_end_states(const orbitsim::GameSimulation &sim,
                                             const std::vector<orbitsim::BodyId> &body_ids,
                                             std::vector<orbitsim::State> &body_states)
        {
            body_states.clear();
            body_states.reserve(body_ids.size());

            for (const orbitsim::BodyId body_id : body_ids)
            {
                const orbitsim::MassiveBody *body = sim.body_by_id(body_id);
                if (!body || !finite_state(body->state))
                {
                    body_states.clear();
                    return false;
                }
                body_states.push_back(body->state);
            }
            return body_states.size() == body_ids.size();
        }

        bool step_runtime_massive_bodies_only(orbitsim::GameSimulation &sim, const double dt_s)
        {
            if (!(dt_s != 0.0) || !std::isfinite(dt_s))
            {
                return true;
            }

            const double G = sim.config().gravitational_constant;
            if (!(G > 0.0) || !std::isfinite(G))
            {
                return false;
            }

            std::vector<orbitsim::MassiveBody> bodies = sim.massive_bodies();
            double t_s = sim.time_s();
            orbitsim::detail::preview_step_massive_bodies(bodies,
                                                          t_s,
                                                          dt_s,
                                                          G,
                                                          sim.config().softening_length_m,
                                                          nullptr);
            for (const orbitsim::MassiveBody &body : bodies)
            {
                if (!sim.has_body(body.id) ||
                    !finite_vec3(body.state.position_m) ||
                    !finite_vec3(body.state.velocity_mps) ||
                    !finite_vec3(body.state.spin.axis) ||
                    !std::isfinite(body.state.spin.angle_rad) ||
                    !std::isfinite(body.state.spin.rate_rad_per_s))
                {
                    return false;
                }
            }
            for (const orbitsim::MassiveBody &body : bodies)
            {
                if (!sim.set_body_state(body.id, body.state))
                {
                    return false;
                }
            }
            return sim.set_time_s(t_s);
        }

        bool body_state_at_runtime(const orbitsim::GameSimulation &sim,
                                   const orbitsim::CelestialEphemeris &ephemeris,
                                   const orbitsim::BodyId body_id,
                                   const double t_s,
                                   orbitsim::State &out_state)
        {
            const orbitsim::MassiveBody *body = sim.body_by_id(body_id);
            if (!body)
            {
                return false;
            }
            out_state = !ephemeris.empty()
                                ? ephemeris.body_state_at_by_id(body_id, t_s)
                                : body->state;
            return finite_state(out_state);
        }

        bool sample_runtime_arc_inertial_state(const orbitsim::GameSimulation &sim,
                                               const orbitsim::CelestialEphemeris &ephemeris,
                                               const orbitsim::KeplerArc &arc,
                                               const double t_s,
                                               const orbitsim::KeplerPropagationOptions &propagation,
                                               orbitsim::State &out_state)
        {
            const orbitsim::KeplerArcSample sample =
                    orbitsim::sample_kepler_arc_state(arc, t_s, propagation);
            if (!sample.ok() || !finite_state(sample.state_relative))
            {
                return false;
            }

            orbitsim::State primary_state{};
            if (!body_state_at_runtime(sim, ephemeris, arc.primary_body_id, t_s, primary_state))
            {
                return false;
            }

            out_state = sample.state_relative;
            out_state.position_m += primary_state.position_m;
            out_state.velocity_mps += primary_state.velocity_mps;
            return finite_state(out_state);
        }

        bool propagate_spacecraft_patched_conics_step(
                const orbitsim::GameSimulation &sim,
                const orbitsim::CelestialEphemeris &ephemeris,
                const orbitsim::State &spacecraft0,
                const orbitsim::BodyId primary0_id,
                const double t0_s,
                const double t1_s,
                const OrbitalPhysicsSystem::Context &context,
                orbitsim::State &out_state,
                orbitsim::BodyId &out_primary_id)
        {
            if (!finite_state(spacecraft0) ||
                primary0_id == orbitsim::kInvalidBodyId ||
                !std::isfinite(t0_s) ||
                !std::isfinite(t1_s) ||
                t1_s <= t0_s)
            {
                return false;
            }

            const double G = sim.config().gravitational_constant;
            if (!(G > 0.0) || !std::isfinite(G))
            {
                return false;
            }

            double cursor_t_s = t0_s;
            orbitsim::State cursor_state = spacecraft0;
            orbitsim::BodyId current_primary_id = primary0_id;

            for (std::size_t patch_index = 0u;
                 patch_index < kRuntimeMaxPatchesPerStep &&
                 cursor_t_s < t1_s - kRuntimePatchTimeEpsilonS;
                 ++patch_index)
            {
                const orbitsim::MassiveBody *primary = sim.body_by_id(current_primary_id);
                if (!primary || !(primary->mass_kg > 0.0) || !std::isfinite(primary->mass_kg))
                {
                    return false;
                }

                orbitsim::State primary_state{};
                if (!body_state_at_runtime(sim, ephemeris, current_primary_id, cursor_t_s, primary_state))
                {
                    return false;
                }

                orbitsim::KeplerArc arc{
                        .mu_m3_s2 = G * primary->mass_kg,
                        .primary_body_id = current_primary_id,
                        .t0_s = cursor_t_s,
                        .t1_s = t1_s,
                        .state0_relative = orbitsim::make_state(
                                cursor_state.position_m - primary_state.position_m,
                                cursor_state.velocity_mps - primary_state.velocity_mps,
                                cursor_state.spin),
                };
                if (!orbitsim::kepler_arc_valid(arc))
                {
                    return false;
                }

                orbitsim::SoiTransitionSearchOptions transition_options{};
                transition_options.max_step_s =
                        runtime_transition_search_step_s(context, cursor_t_s, t1_s);
                transition_options.refine_tolerance_s = kRuntimePatchRefineToleranceS;
                transition_options.switch_options = context.soi_switch_options;
                transition_options.propagation = context.kepler_propagation;

                const orbitsim::SoiTransitionSearchResult transition =
                        orbitsim::find_next_soi_transition_on_kepler_arc(sim,
                                                                         ephemeris,
                                                                         arc,
                                                                         current_primary_id,
                                                                         t1_s,
                                                                         transition_options);
                if (transition.first_failure != orbitsim::KeplerStatus::Ok)
                {
                    return false;
                }
                if (transition.budget_hit)
                {
                    return false;
                }

                double cut_t_s = t1_s;
                orbitsim::BodyId next_primary_id = current_primary_id;
                if (transition.found &&
                    transition.to_primary_body_id != orbitsim::kInvalidBodyId &&
                    transition.t_s > cursor_t_s + kRuntimePatchTimeEpsilonS &&
                    transition.t_s < t1_s - kRuntimePatchTimeEpsilonS)
                {
                    cut_t_s = transition.t_s;
                    next_primary_id = transition.to_primary_body_id;
                }

                orbitsim::State cut_state{};
                if (!sample_runtime_arc_inertial_state(sim,
                                                       ephemeris,
                                                       arc,
                                                       cut_t_s,
                                                       context.kepler_propagation,
                                                       cut_state))
                {
                    return false;
                }

                cursor_state = cut_state;
                cursor_t_s = cut_t_s;
                current_primary_id = next_primary_id;

                if (cut_t_s >= t1_s - kRuntimePatchTimeEpsilonS)
                {
                    out_state = cursor_state;
                    out_primary_id = current_primary_id;
                    return true;
                }
            }

            if (cursor_t_s >= t1_s - kRuntimePatchTimeEpsilonS)
            {
                out_state = cursor_state;
                out_primary_id = current_primary_id;
                return true;
            }
            return false;
        }

        const orbitsim::MassiveBody *cached_or_selected_soi_primary_body(
                const OrbitalScenario &scenario,
                const glm::dvec3 &spacecraft_position_m,
                orbitsim::BodyId *cached_primary_body_id,
                const orbitsim::SoiSwitchOptions &options)
        {
            if (cached_primary_body_id && *cached_primary_body_id != orbitsim::kInvalidBodyId)
            {
                const orbitsim::MassiveBody *cached = scenario.sim.body_by_id(*cached_primary_body_id);
                if (cached &&
                    cached->mass_kg > 0.0 &&
                    std::isfinite(cached->mass_kg) &&
                    finite_state(cached->state) &&
                    cached->soi_radius_m > 0.0 &&
                    std::isfinite(cached->soi_radius_m))
                {
                    const double exit_scale =
                            std::isfinite(options.exit_scale) ? std::max(0.0, options.exit_scale) : 0.0;
                    const double keep_radius_m = exit_scale * cached->soi_radius_m;
                    const double distance_m =
                            glm::length(spacecraft_position_m - glm::dvec3(cached->state.position_m));
                    if (std::isfinite(distance_m) && distance_m <= keep_radius_m)
                    {
                        return cached;
                    }
                }
            }

            return select_soi_primary_body(scenario,
                                           spacecraft_position_m,
                                           cached_primary_body_id,
                                           options);
        }

        glm::dvec3 spacecraft_gravity_accel_body_centered(const OrbitalPhysicsSystem::Context &context,
                                                          const glm::dvec3 &p_rel_m,
                                                          orbitsim::BodyId *primary_body_id)
        {
            OrbitalScenario *scenario = context.orbit.scenario_owner().get();
            if (!scenario)
            {
                return glm::dvec3(0.0);
            }

            switch (context.spacecraft_gravity_mode)
            {
                case SpacecraftGravityMode::NBody:
                    return nbody_accel_body_centered(*scenario, p_rel_m);

                case SpacecraftGravityMode::SoiKepler:
                    return soi_accel_body_centered(*scenario,
                                                  p_rel_m,
                                                  primary_body_id,
                                                  context.soi_switch_options);
            }
            return glm::dvec3(0.0);
        }

        void step_orbitsim_with_runtime_spacecraft_mode(
                const OrbitalPhysicsSystem::Context &context,
                const double dt_s)
        {
            OrbitalScenario *scenario = context.orbit.scenario_owner().get();
            if (!scenario)
            {
                return;
            }

            if (context.spacecraft_gravity_mode != SpacecraftGravityMode::SoiKepler)
            {
                scenario->sim.step(dt_s);
                return;
            }

            struct Patch
            {
                orbitsim::SpacecraftId spacecraft_id{orbitsim::kInvalidSpacecraftId};
                orbitsim::BodyId primary_id{orbitsim::kInvalidBodyId};
                orbitsim::BodyId *primary_cache{nullptr};
                orbitsim::State spacecraft0{};
                orbitsim::State primary0{};
                double mu{};
            };

            const double G = scenario->sim.config().gravitational_constant;
            const double max_step_s =
                    (std::isfinite(context.soi_kepler_max_step_s) && context.soi_kepler_max_step_s > 0.0)
                            ? context.soi_kepler_max_step_s
                            : 60.0;
            std::vector<Patch> patches;
            patches.reserve(context.orbit.orbiters().size());
            std::vector<orbitsim::BodyId> body_ids;
            std::vector<orbitsim::State> body_start_states;
            std::vector<orbitsim::State> body_end_states;
            orbitsim::CelestialEphemeris step_ephemeris{};

            double remaining_s = dt_s;
            while (std::isfinite(remaining_s) && remaining_s != 0.0)
            {
                const double step_s = std::copysign(std::min(std::abs(remaining_s), max_step_s), remaining_s);
                const double t0_s = scenario->sim.time_s();
                patches.clear();
                body_ids.clear();
                body_start_states.clear();
                body_end_states.clear();
                step_ephemeris = {};
                bool step_ephemeris_ready = false;

                if (std::isfinite(G) && G > 0.0)
                {
                    for (OrbiterInfo &orbiter : context.orbit.orbiters())
                    {
                        const orbitsim::Spacecraft *spacecraft =
                                orbiter.rails.active()
                                        ? scenario->sim.spacecraft_by_id(orbiter.rails.sc_id)
                                        : nullptr;
                        if (!spacecraft || !finite_state(spacecraft->state))
                        {
                            continue;
                        }

                        const orbitsim::MassiveBody *primary =
                                cached_or_selected_soi_primary_body(*scenario,
                                                                    spacecraft->state.position_m,
                                                                    &orbiter.soi_kepler_primary_body_id,
                                                                    context.soi_switch_options);
                        if (!primary || !finite_state(primary->state))
                        {
                            continue;
                        }

                        patches.push_back(Patch{.spacecraft_id = spacecraft->id,
                                                .primary_id = primary->id,
                                                .primary_cache = &orbiter.soi_kepler_primary_body_id,
                                                .spacecraft0 = spacecraft->state,
                                                .primary0 = primary->state,
                                                .mu = G * primary->mass_kg});
                    }

                    if (!patches.empty())
                    {
                        capture_runtime_body_start_states(scenario->sim,
                                                          body_ids,
                                                          body_start_states);
                    }
                }

                if (!step_runtime_massive_bodies_only(scenario->sim, step_s))
                {
                    scenario->sim.step(step_s);
                }

                const double t1_s = t0_s + step_s;
                for (const Patch &patch : patches)
                {
                    const orbitsim::MassiveBody *primary = scenario->sim.body_by_id(patch.primary_id);
                    if (!primary || !(patch.mu > 0.0))
                    {
                        continue;
                    }

                    const orbitsim::KeplerArc arc{
                            .mu_m3_s2 = patch.mu,
                            .primary_body_id = patch.primary_id,
                            .t0_s = t0_s,
                            .t1_s = t1_s,
                            .state0_relative = orbitsim::make_state(
                                    patch.spacecraft0.position_m - patch.primary0.position_m,
                                    patch.spacecraft0.velocity_mps - patch.primary0.velocity_mps,
                                    patch.spacecraft0.spin),
                    };
                    orbitsim::KeplerArcSample sample =
                            orbitsim::sample_kepler_arc_state(arc, t1_s, context.kepler_propagation);
                    if (!sample.ok())
                    {
                        continue;
                    }

                    orbitsim::State end_state = sample.state_relative;
                    end_state.position_m += primary->state.position_m;
                    end_state.velocity_mps += primary->state.velocity_mps;
                    if (!finite_state(end_state))
                    {
                        continue;
                    }

                    if (!runtime_has_other_soi_candidate(scenario->sim, patch.primary_id))
                    {
                        (void) scenario->sim.set_spacecraft_state(patch.spacecraft_id, end_state);
                        continue;
                    }

                    if (!step_ephemeris_ready)
                    {
                        if (!capture_runtime_body_end_states(scenario->sim,
                                                             body_ids,
                                                             body_end_states))
                        {
                            (void) scenario->sim.set_spacecraft_state(patch.spacecraft_id, end_state);
                            continue;
                        }
                        step_ephemeris = make_runtime_step_ephemeris(body_ids,
                                                                     body_start_states,
                                                                     body_end_states,
                                                                     t0_s,
                                                                     t1_s);
                        step_ephemeris_ready = !step_ephemeris.empty();
                        if (!step_ephemeris_ready)
                        {
                            (void) scenario->sim.set_spacecraft_state(patch.spacecraft_id, end_state);
                            continue;
                        }
                    }

                    orbitsim::State patched_state{};
                    orbitsim::BodyId patched_primary_id = patch.primary_id;
                    if (propagate_spacecraft_patched_conics_step(scenario->sim,
                                                                 step_ephemeris,
                                                                 patch.spacecraft0,
                                                                 patch.primary_id,
                                                                 t0_s,
                                                                 t1_s,
                                                                 context,
                                                                 patched_state,
                                                                 patched_primary_id))
                    {
                        (void) scenario->sim.set_spacecraft_state(patch.spacecraft_id, patched_state);
                        if (patch.primary_cache && patched_primary_id != orbitsim::kInvalidBodyId)
                        {
                            *patch.primary_cache = patched_primary_id;
                        }
                    }
                    else
                    {
                        (void) scenario->sim.set_spacecraft_state(patch.spacecraft_id, end_state);
                    }
                }
                remaining_s -= step_s;
            }
        }

        void update_rebase_anchor(GameWorld &world, const OrbitalRuntimeSystem &orbit)
        {
            const EntityId next_anchor = orbit.select_rebase_anchor_entity();
            if (!next_anchor.is_valid())
            {
                world.clear_rebase_anchor();
                return;
            }

            if (next_anchor != world.rebase_anchor())
            {
                world.set_rebase_anchor(next_anchor);
            }
        }

        // Integrates rails-warp angular velocity and orientation using gameplay-tuned torque/SAS rules.
        void update_rails_rotation(OrbiterInfo::RailsState &rs,
                                   const glm::vec3 &world_torque_dir,
                                   const float torque_strength,
                                   const float sas_damping,
                                   const bool sas_enabled,
                                   const double dt_s)
        {
            const float dt = static_cast<float>(dt_s);
            if (!(dt > 0.0f) || !std::isfinite(dt))
            {
                return;
            }

            if (glm::length(world_torque_dir) > 0.0f)
            {
                rs.angular_velocity_radps += world_torque_dir * (torque_strength * dt);
            }

            if (sas_enabled && glm::length(world_torque_dir) < 0.01f)
            {
                const float damping = std::max(0.0f, sas_damping);
                const float decay = std::exp(-damping * dt);
                rs.angular_velocity_radps *= decay;
                if (glm::length(rs.angular_velocity_radps) < 1e-3f)
                {
                    rs.angular_velocity_radps = glm::vec3(0.0f);
                }
            }

            const float omega = glm::length(rs.angular_velocity_radps);
            if (omega > 1e-6f)
            {
                const float angle = omega * dt;
                const glm::vec3 axis = rs.angular_velocity_radps / omega;
                rs.rotation = glm::angleAxis(angle, axis) * rs.rotation;
                rs.rotation = glm::normalize(rs.rotation);
            }
        }
    } // namespace

    void OrbitalPhysicsSystem::reset()
    {
        _rails_warp_active = false;
        _last_sim_step_dt_s = 0.0;
        _rails_thrust_applied_this_tick = false;
        _rails_last_thrust_dir_local = glm::vec3(0.0f);
        _rails_last_torque_dir_local = glm::vec3(0.0f);
    }

    void OrbitalPhysicsSystem::sync_celestial_render_entities(const Context &context, GameStateContext &ctx)
    {
        OrbitalRuntimeSystem &orbit = context.orbit;
        if (!orbit.scenario_owner() || orbit.scenario_owner()->bodies.empty())
        {
            return;
        }

        const auto &cfg = context.scenario_config;

        const CelestialBodyInfo *ref_info = orbit.scenario_owner()->world_reference_body();
        const orbitsim::MassiveBody *ref_sim = orbit.scenario_owner()->world_reference_sim_body();
        if (!ref_info || !ref_sim)
        {
            return;
        }

        for (auto &body_info : orbit.scenario_owner()->bodies)
        {
            if (body_info.sim_id == ref_info->sim_id)
            {
                continue;
            }

            const orbitsim::MassiveBody *sim_body = orbit.scenario_owner()->sim.body_by_id(body_info.sim_id);
            if (!sim_body)
            {
                continue;
            }

            const WorldVec3 body_pos_world =
                    cfg.system_center + WorldVec3(sim_body->state.position_m - ref_sim->state.position_m);

            if (ctx.api)
            {
                (void) ctx.api->set_planet_center(body_info.name, glm::dvec3(body_pos_world));
            }

            if (!body_info.render_entity.is_valid())
            {
                continue;
            }

            if (Entity *ent = context.world.entities().find(body_info.render_entity))
            {
                ent->set_position_world(body_pos_world);
                ent->set_rotation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
            }
        }
    }

    void OrbitalPhysicsSystem::update_runtime_orbiter_rails(const Context &context)
    {
        if (_rails_warp_active)
        {
            return;
        }

        OrbitalRuntimeSystem &orbit = context.orbit;
        const bool can_run_runtime_rails =
                orbit.runtime_orbiter_rails_enabled() &&
                orbit.runtime_orbiter_rails_distance_m() > 0.0 &&
                orbit.scenario_owner() &&
                context.physics &&
                context.physics_context &&
                orbit.scenario_owner()->world_reference_sim_body();

        const EntityId anchor_eid = orbit.select_rebase_anchor_entity();
        const Entity *anchor_entity = context.world.entities().find(anchor_eid);
        const WorldVec3 anchor_pos_world =
                anchor_entity ? anchor_entity->physics_center_of_mass_world() : context.scenario_config.system_center;

        const double promote_distance_m = std::max(0.0, orbit.runtime_orbiter_rails_distance_m());
        const double return_distance_m = promote_distance_m * kRuntimeOrbiterRailsReturnDistanceRatio;

        for (auto &orbiter : orbit.orbiters())
        {
            if (orbiter.is_player || !orbiter.entity.is_valid())
            {
                continue;
            }

            WorldVec3 orbiter_pos_world{0.0, 0.0, 0.0};
            if (orbiter.rails.active() && orbit.scenario_owner())
            {
                const orbitsim::MassiveBody *ref_sim = orbit.scenario_owner()->world_reference_sim_body();
                const orbitsim::Spacecraft *sc = ref_sim ? orbit.scenario_owner()->sim.spacecraft_by_id(orbiter.rails.sc_id) : nullptr;
                if (ref_sim && sc)
                {
                    orbiter_pos_world = context.scenario_config.system_center +
                            WorldVec3(sc->state.position_m - ref_sim->state.position_m);
                }
                else if (const Entity *entity = context.world.entities().find(orbiter.entity))
                {
                    orbiter_pos_world = entity->physics_center_of_mass_world();
                }
            }
            else if (const Entity *entity = context.world.entities().find(orbiter.entity))
            {
                orbiter_pos_world = entity->physics_center_of_mass_world();
            }

            const double distance_m = glm::length(glm::dvec3(orbiter_pos_world - anchor_pos_world));

            if (!can_run_runtime_rails)
            {
                if (orbiter.rails.active())
                {
                    (void) demote_orbiter_from_rails(context, orbiter);
                }
                continue;
            }

            if (orbiter.rails.active())
            {
                if (distance_m <= return_distance_m)
                {
                    (void) demote_orbiter_from_rails(context, orbiter);
                }
                continue;
            }

            if (distance_m >= promote_distance_m)
            {
                (void) promote_orbiter_to_rails(context, orbiter);
            }
        }
    }

    void OrbitalPhysicsSystem::sync_runtime_orbiter_rails(const Context &context, const double dt_s)
    {
        OrbitalRuntimeSystem &orbit = context.orbit;
        if (_rails_warp_active || !orbit.scenario_owner())
        {
            return;
        }

        const orbitsim::MassiveBody *ref_sim = orbit.scenario_owner()->world_reference_sim_body();
        if (!ref_sim)
        {
            return;
        }

        for (auto &orbiter : orbit.orbiters())
        {
            if (!orbiter.rails.active() || !orbiter.entity.is_valid())
            {
                continue;
            }

            const orbitsim::Spacecraft *sc = orbit.scenario_owner()->sim.spacecraft_by_id(orbiter.rails.sc_id);
            Entity *ent = context.world.entities().find(orbiter.entity);
            if (!sc || !ent)
            {
                continue;
            }

            update_rails_rotation(orbiter.rails,
                                  glm::vec3(0.0f),
                                  0.0f,
                                  0.0f,
                                  false,
                                  dt_s);

            const WorldVec3 body_pos_world = context.scenario_config.system_center +
                    WorldVec3(sc->state.position_m - ref_sim->state.position_m);
            const WorldVec3 entity_pos_world =
                    OrbiterPhysicsBridge::entity_world_from_body_world(*ent, body_pos_world, orbiter.rails.rotation);

            ent->set_position_world(entity_pos_world);
            ent->set_rotation(orbiter.rails.rotation);
            if (ent->uses_interpolation())
            {
                ent->interpolation().curr_position = entity_pos_world;
                ent->interpolation().curr_rotation = orbiter.rails.rotation;
            }
        }
    }

    bool OrbitalPhysicsSystem::promote_orbiter_to_rails(const Context &context, OrbiterInfo &orbiter)
    {
        OrbitalRuntimeSystem &orbit = context.orbit;
        if (_rails_warp_active || !orbit.scenario_owner() || !context.physics_context || !orbiter.entity.is_valid() ||
            orbiter.rails.active())
        {
            return false;
        }

        const orbitsim::MassiveBody *ref_sim = orbit.scenario_owner()->world_reference_sim_body();
        Entity *ent = context.world.entities().find(orbiter.entity);
        if (!ref_sim || !ent || !ent->has_physics() || !context.physics)
        {
            return false;
        }

        const Physics::BodyId body_id{ent->physics_body_value()};
        if (!context.physics->is_body_valid(body_id))
        {
            return false;
        }

        const OrbiterPhysicsBridgeContext bridge_ctx{
            .renderer = context.renderer,
            .world = &context.world,
            .physics = context.physics,
            .physics_context = context.physics_context,
        };
        OrbiterBodyState state{};
        if (!OrbiterPhysicsBridge::read_body_state(bridge_ctx, *ent, state))
        {
            return false;
        }

        orbitsim::Spacecraft sc{};
        sc.state = orbitsim::make_state(
                ref_sim->state.position_m + glm::dvec3(state.body_position_world - context.scenario_config.system_center),
                ref_sim->state.velocity_mps + state.velocity_world);
        sc.dry_mass_kg = std::max(1.0, orbiter.mass_kg);

        const auto handle = orbit.scenario_owner()->sim.create_spacecraft(std::move(sc));
        if (!handle.valid())
        {
            return false;
        }

        (void) OrbiterPhysicsBridge::destroy_body(bridge_ctx, orbiter.render_is_gltf, *ent);

        orbiter.rails.sc_id = handle.id;
        orbiter.rails.rotation = state.rotation;
        orbiter.rails.angular_velocity_radps = state.angular_velocity_world;
        orbiter.rails.sas_enabled = false;
        orbiter.rails.sas_toggle_prev_down = false;

        ent->set_position_world(
                OrbiterPhysicsBridge::entity_world_from_body_world(*ent, state.body_position_world, state.rotation));
        ent->set_rotation(state.rotation);
        if (ent->uses_interpolation())
        {
            ent->interpolation().set_immediate(ent->position_world(), state.rotation);
        }
        return true;
    }

    bool OrbitalPhysicsSystem::demote_orbiter_from_rails(const Context &context, OrbiterInfo &orbiter)
    {
        OrbitalRuntimeSystem &orbit = context.orbit;
        if (_rails_warp_active || !orbit.scenario_owner() || !context.physics || !context.physics_context ||
            !orbiter.rails.active() || !orbiter.entity.is_valid())
        {
            return false;
        }

        const orbitsim::MassiveBody *ref_sim = orbit.scenario_owner()->world_reference_sim_body();
        const orbitsim::Spacecraft *sc = ref_sim ? orbit.scenario_owner()->sim.spacecraft_by_id(orbiter.rails.sc_id) : nullptr;
        Entity *ent = context.world.entities().find(orbiter.entity);
        if (!ref_sim || !sc || !ent)
        {
            return false;
        }

        const WorldVec3 body_pos_world = context.scenario_config.system_center +
                WorldVec3(sc->state.position_m - ref_sim->state.position_m);
        const glm::dvec3 vel_world = sc->state.velocity_mps - ref_sim->state.velocity_mps;
        const glm::quat rot = orbiter.rails.rotation;
        const OrbiterPhysicsBridgeContext bridge_ctx{
            .renderer = context.renderer,
            .world = &context.world,
            .physics = context.physics,
            .physics_context = context.physics_context,
        };
        if (!OrbiterPhysicsBridge::restore_body_state(bridge_ctx,
                                                      orbiter.render_is_gltf,
                                                      *ent,
                                                      orbiter.physics_settings,
                                                      orbiter.use_physics_interpolation,
                                                      OrbiterBodyState{
                                                          .body_position_world = body_pos_world,
                                                          .velocity_world = vel_world,
                                                          .rotation = rot,
                                                          .angular_velocity_world = orbiter.rails.angular_velocity_radps,
                                                      }))
        {
            return false;
        }

        (void) orbit.scenario_owner()->sim.remove_spacecraft(orbiter.rails.sc_id);
        orbiter.rails.clear();
        return true;
    }

    void OrbitalPhysicsSystem::update_formation_hold(const Context &context, const double dt_s)
    {
        FormationHoldSystem::update(FormationHoldSystem::Context{
                                            .orbit = context.orbit,
                                            .world = context.world,
                                            .physics = context.physics,
                                            .physics_context = context.physics_context,
                                            .system_center = context.scenario_config.system_center,
                                            .omega = kFormationHoldOmega,
                                            .max_dv_per_step_mps = kFormationHoldMaxDvPerStepMps,
                                            .world_state_sampler = context.orbiter_world_state_sampler,
                                    },
                                    dt_s);
    }

    void OrbitalPhysicsSystem::step_physics(const Context &context, GameStateContext &ctx, const float fixed_dt)
    {
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (!context.physics || !context.physics_context)
        {
            return;
        }

        update_rebase_anchor(context.world, context.orbit);
        context.world.pre_physics_step();

        const auto &cfg = context.scenario_config;
        const bool use_orbitsim = context.orbit.scenario_owner() && !context.orbit.scenario_owner()->bodies.empty()
                                  && context.orbit.scenario_owner()->world_reference_body() != nullptr;

        if (use_orbitsim)
        {
            update_runtime_orbiter_rails(context);
            step_orbitsim_with_runtime_spacecraft_mode(context, static_cast<double>(fixed_dt));
            sync_celestial_render_entities(context, ctx);
            sync_runtime_orbiter_rails(context, static_cast<double>(fixed_dt));
        }

        auto gravity_accel_world_at = [&](const WorldVec3 &p_world,
                                          orbitsim::BodyId *primary_body_id) -> glm::dvec3 {
            if (!use_orbitsim)
            {
                return glm::dvec3(0.0);
            }

            const glm::dvec3 p_rel = glm::dvec3(p_world - cfg.system_center);
            return spacecraft_gravity_accel_body_centered(context, p_rel, primary_body_id);
        };

        glm::dvec3 anchor_accel_world(0.0);
        bool have_anchor_accel = false;
        const bool per_step_sync = _velocity_origin_mode == VelocityOriginMode::PerStepAnchorSync;
        const WorldVec3 physics_origin_world = context.physics_context->origin_world();

        auto primary_cache_for_entity = [&](const EntityId entity) -> orbitsim::BodyId * {
            for (OrbiterInfo &orbiter : context.orbit.orbiters())
            {
                if (orbiter.entity == entity)
                {
                    return &orbiter.soi_kepler_primary_body_id;
                }
            }
            return nullptr;
        };

        EntityId anchor_eid = context.world.rebase_anchor();
        if (!anchor_eid.is_valid())
        {
            anchor_eid = context.orbit.player_entity();
        }

        if (anchor_eid.is_valid())
        {
            Entity *anchor = context.world.entities().find(anchor_eid);
            if (anchor && anchor->has_physics())
            {
                const Physics::BodyId anchor_body{anchor->physics_body_value()};
                if (context.physics->is_body_valid(anchor_body))
                {
                    const double dt = static_cast<double>(fixed_dt);
                    if (per_step_sync)
                    {
                        const glm::vec3 v_local_f = context.physics->get_linear_velocity(anchor_body);
                        const glm::dvec3 v_world = context.physics_context->velocity_origin_world() + glm::dvec3(v_local_f);
                        (void) context.physics_context->set_velocity_origin_world(v_world);
                        context.physics->shift_velocity_origin(glm::dvec3(v_local_f));
                    }
                    else
                    {
                        const glm::dvec3 p_local_anchor = context.physics->get_position(anchor_body);
                        const glm::quat anchor_rotation = context.physics->get_rotation(anchor_body);
                        const WorldVec3 body_origin_world = physics_origin_world + WorldVec3(p_local_anchor);
                        const WorldVec3 p_world_anchor =
                                anchor->physics_center_of_mass_world(body_origin_world, anchor_rotation);
                        anchor_accel_world = gravity_accel_world_at(p_world_anchor,
                                                                    primary_cache_for_entity(anchor_eid));
                        have_anchor_accel = true;

                        const glm::dvec3 v_origin_next =
                                context.physics_context->velocity_origin_world() + anchor_accel_world * dt;
                        (void) context.physics_context->set_velocity_origin_world(v_origin_next);
                    }
                }
            }
        }

        const glm::dvec3 frame_accel_world =
                (!per_step_sync && have_anchor_accel) ? anchor_accel_world : glm::dvec3(0.0);

        auto apply_gravity_accel = [&](OrbiterInfo &orbiter) {
            Entity *ent = context.world.entities().find(orbiter.entity);
            if (!ent || !ent->has_physics())
            {
                return;
            }

            const Physics::BodyId body_id{ent->physics_body_value()};
            if (!context.physics->is_body_valid(body_id))
            {
                return;
            }

            const glm::dvec3 p_local = context.physics->get_position(body_id);
            const glm::quat rotation = context.physics->get_rotation(body_id);
            const WorldVec3 body_origin_world =
                    OrbiterPhysicsBridge::body_world_from_body_local(p_local, physics_origin_world);
            const WorldVec3 p_world = ent->physics_center_of_mass_world(body_origin_world, rotation);

            const glm::dvec3 a_local =
                    gravity_accel_world_at(p_world, &orbiter.soi_kepler_primary_body_id) - frame_accel_world;

            glm::vec3 v_local = context.physics->get_linear_velocity(body_id);
            v_local += glm::vec3(a_local) * fixed_dt;
            context.physics->set_linear_velocity(body_id, v_local);
            context.physics->activate(body_id);
        };

        for (OrbiterInfo &orbiter : context.orbit.orbiters())
        {
            if (orbiter.apply_gravity && orbiter.entity.is_valid())
            {
                apply_gravity_accel(orbiter);
            }
        }

        context.physics->step(fixed_dt);

        const glm::dvec3 v_origin = context.physics_context->velocity_origin_world();
        if (finite_vec3(v_origin))
        {
            const double dt = static_cast<double>(fixed_dt);
            (void) context.physics_context->set_origin_world(
                    context.physics_context->origin_world() + WorldVec3(v_origin * dt));
        }

        context.world.post_physics_step();
#else
        (void) context;
        (void) ctx;
        (void) fixed_dt;
#endif
    }

    bool OrbitalPhysicsSystem::enter_rails_warp(const Context &context, GameStateContext &ctx)
    {
        if (_rails_warp_active)
        {
            return true;
        }

        OrbitalRuntimeSystem &orbit = context.orbit;
        if (!orbit.scenario_owner())
        {
            return false;
        }

        const orbitsim::MassiveBody *ref_sim = orbit.scenario_owner()->world_reference_sim_body();
        if (!ref_sim)
        {
            return false;
        }

        bool have_player_sc = false;
        std::vector<orbitsim::SpacecraftId> created_ids;
        created_ids.reserve(orbit.orbiters().size());

        const bool sas_down = ctx.input && ctx.input->key_down(Key::T);

        for (auto &orbiter : orbit.orbiters())
        {
            WorldVec3 body_pos_world{0.0, 0.0, 0.0};
            glm::dvec3 vel_world(0.0);
            glm::quat rot{1.0f, 0.0f, 0.0f, 0.0f};
            glm::vec3 ang_vel_world(0.0f);
            bool have_state_snapshot = false;

            if (orbiter.rails.active())
            {
                if (const orbitsim::Spacecraft *sc = orbit.scenario_owner()->sim.spacecraft_by_id(orbiter.rails.sc_id))
                {
                    body_pos_world = context.scenario_config.system_center +
                            WorldVec3(sc->state.position_m - ref_sim->state.position_m);
                    vel_world = sc->state.velocity_mps - ref_sim->state.velocity_mps;
                    rot = orbiter.rails.rotation;
                    ang_vel_world = orbiter.rails.angular_velocity_radps;
                    have_state_snapshot = true;
                }
                (void) orbit.scenario_owner()->sim.remove_spacecraft(orbiter.rails.sc_id);
                orbiter.rails.clear();
            }

            if (!orbiter.entity.is_valid())
            {
                continue;
            }

            Entity *ent = context.world.entities().find(orbiter.entity);
            if (!ent)
            {
                continue;
            }

            if (!have_state_snapshot)
            {
                body_pos_world =
                        OrbiterPhysicsBridge::body_world_from_entity_world(*ent, ent->position_world(), ent->rotation());
                rot = ent->rotation();
            }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
            if (context.physics && context.physics_context && ent->has_physics())
            {
                const Physics::BodyId body_id{ent->physics_body_value()};
                if (context.physics->is_body_valid(body_id))
                {
                    rot = context.physics->get_rotation(body_id);
                    const WorldVec3 body_origin_world =
                            OrbiterPhysicsBridge::body_world_from_body_local(context.physics->get_position(body_id),
                                                                             context.physics_context->origin_world());
                    body_pos_world = OrbiterPhysicsBridge::body_world_from_entity_world(*ent, body_origin_world, rot);
                    const glm::vec3 v_local_f = context.physics->get_linear_velocity(body_id);
                    vel_world = context.physics_context->velocity_origin_world() + glm::dvec3(v_local_f);
                    ang_vel_world = context.physics->get_angular_velocity(body_id);
                }
            }
#endif

            const glm::dvec3 rel_pos_m = glm::dvec3(body_pos_world - context.scenario_config.system_center);
            const glm::dvec3 rel_vel_mps = vel_world;

            orbitsim::Spacecraft sc{};
            sc.state = orbitsim::make_state(ref_sim->state.position_m + rel_pos_m,
                                            ref_sim->state.velocity_mps + rel_vel_mps);
            sc.dry_mass_kg = std::max(1.0, orbiter.mass_kg);

            const auto handle = orbit.scenario_owner()->sim.create_spacecraft(std::move(sc));
            if (!handle.valid())
            {
                continue;
            }

            orbiter.rails.sc_id = handle.id;
            orbiter.rails.rotation = rot;
            orbiter.rails.angular_velocity_radps = ang_vel_world;
            orbiter.rails.sas_enabled = false;
            orbiter.rails.sas_toggle_prev_down = sas_down;

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
            if (orbiter.is_player)
            {
                if (auto *sc_comp = ent->get_component<ShipController>())
                {
                    orbiter.rails.sas_enabled = sc_comp->sas_enabled();
                }
            }
#endif

            created_ids.push_back(handle.id);

            if (orbiter.is_player)
            {
                have_player_sc = true;
            }
        }

        if (!have_player_sc)
        {
            for (orbitsim::SpacecraftId id : created_ids)
            {
                (void) orbit.scenario_owner()->sim.remove_spacecraft(id);
            }
            for (auto &orbiter : orbit.orbiters())
            {
                orbiter.rails.clear();
            }
            return false;
        }

        _rails_last_thrust_dir_local = glm::vec3(0.0f);
        _rails_last_torque_dir_local = glm::vec3(0.0f);
        _rails_thrust_applied_this_tick = false;
        _rails_warp_active = true;
        return true;
    }

    void OrbitalPhysicsSystem::exit_rails_warp(const Context &context, GameStateContext &ctx)
    {
        if (!_rails_warp_active)
        {
            return;
        }

        OrbitalRuntimeSystem &orbit = context.orbit;
        if (!orbit.scenario_owner())
        {
            for (auto &orbiter : orbit.orbiters())
            {
                orbiter.rails.clear();
            }
            _rails_warp_active = false;
            return;
        }

        const orbitsim::MassiveBody *ref_sim = orbit.scenario_owner()->world_reference_sim_body();
        if (!ref_sim)
        {
            for (auto &orbiter : orbit.orbiters())
            {
                if (orbiter.rails.active())
                {
                    (void) orbit.scenario_owner()->sim.remove_spacecraft(orbiter.rails.sc_id);
                }
                orbiter.rails.clear();
            }
            _rails_warp_active = false;
            return;
        }

        WorldVec3 anchor_pos_world = context.scenario_config.system_center;
        glm::dvec3 anchor_vel_world(0.0);

        {
            const EntityId anchor_eid = orbit.select_rebase_anchor_entity();
            const OrbiterInfo *anchor_orbiter = nullptr;
            for (const auto &o : orbit.orbiters())
            {
                if (o.entity == anchor_eid)
                {
                    anchor_orbiter = &o;
                    break;
                }
            }

            if (!anchor_orbiter)
            {
                anchor_orbiter = orbit.find_player_orbiter();
            }

            if (anchor_orbiter && anchor_orbiter->rails.active())
            {
                if (const orbitsim::Spacecraft *sc = orbit.scenario_owner()->sim.spacecraft_by_id(anchor_orbiter->rails.sc_id))
                {
                    anchor_pos_world = context.scenario_config.system_center +
                            WorldVec3(sc->state.position_m - ref_sim->state.position_m);
                    anchor_vel_world = sc->state.velocity_mps - ref_sim->state.velocity_mps;
                }
            }
        }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (context.physics_context)
        {
            (void) context.physics_context->set_origin_world(anchor_pos_world);
            (void) context.physics_context->set_velocity_origin_world(anchor_vel_world);
        }
#endif

        for (auto &orbiter : orbit.orbiters())
        {
            if (!orbiter.rails.active())
            {
                continue;
            }

            const orbitsim::Spacecraft *sc = orbit.scenario_owner()->sim.spacecraft_by_id(orbiter.rails.sc_id);
            if (!sc)
            {
                continue;
            }

            const WorldVec3 body_pos_world = context.scenario_config.system_center +
                    WorldVec3(sc->state.position_m - ref_sim->state.position_m);
            const glm::dvec3 vel_world = sc->state.velocity_mps - ref_sim->state.velocity_mps;
            const glm::quat rot = orbiter.rails.rotation;

            Entity *ent = context.world.entities().find(orbiter.entity);
            const WorldVec3 entity_pos_world =
                    ent ? OrbiterPhysicsBridge::entity_world_from_body_world(*ent, body_pos_world, rot)
                        : WorldVec3(0.0);
            if (ent)
            {
                ent->set_position_world(entity_pos_world);
                ent->set_rotation(rot);
                if (ent->uses_interpolation())
                {
                    ent->interpolation().set_immediate(entity_pos_world, rot);
                }
            }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
            if (context.physics && context.physics_context && ent)
            {
                const OrbiterPhysicsBridgeContext bridge_ctx{
                    .renderer = context.renderer,
                    .world = &context.world,
                    .physics = context.physics,
                    .physics_context = context.physics_context,
                };
                (void) OrbiterPhysicsBridge::restore_body_state(bridge_ctx,
                                                                orbiter.render_is_gltf,
                                                                *ent,
                                                                orbiter.physics_settings,
                                                                orbiter.use_physics_interpolation,
                                                                OrbiterBodyState{
                                                                    .body_position_world = body_pos_world,
                                                                    .velocity_world = vel_world,
                                                                    .rotation = rot,
                                                                    .angular_velocity_world =
                                                                            orbiter.rails.angular_velocity_radps,
                                                                });
            }

            if (orbiter.is_player && ent)
            {
                if (auto *sc_comp = ent->get_component<ShipController>())
                {
                    sc_comp->set_sas_enabled(orbiter.rails.sas_enabled);
                    const bool sas_down = ctx.input && ctx.input->key_down(Key::T);
                    sc_comp->set_sas_toggle_prev_down(sas_down);
                }
            }
#endif
        }

        for (auto &orbiter : orbit.orbiters())
        {
            if (orbiter.rails.active())
            {
                (void) orbit.scenario_owner()->sim.remove_spacecraft(orbiter.rails.sc_id);
            }
            orbiter.rails.clear();
        }

        _rails_last_thrust_dir_local = glm::vec3(0.0f);
        _rails_last_torque_dir_local = glm::vec3(0.0f);
        _rails_thrust_applied_this_tick = false;
        _rails_warp_active = false;
    }

    void OrbitalPhysicsSystem::rails_warp_step(const Context &context, GameStateContext &ctx, const double dt_s)
    {
        _rails_thrust_applied_this_tick = false;
        _rails_last_thrust_dir_local = glm::vec3(0.0f);
        _rails_last_torque_dir_local = glm::vec3(0.0f);

        OrbitalRuntimeSystem &orbit = context.orbit;
        if (!orbit.scenario_owner())
        {
            return;
        }

        const orbitsim::MassiveBody *ref_sim = orbit.scenario_owner()->world_reference_sim_body();
        if (!ref_sim)
        {
            return;
        }

        OrbiterInfo *player_orbiter = nullptr;
        for (auto &o : orbit.orbiters())
        {
            if (o.is_player)
            {
                player_orbiter = &o;
                break;
            }
        }

        const bool ui_capture_keyboard = context.ui_capture_keyboard && context.ui_capture_keyboard(ctx);

        float thrust_force_N = 0.0f;
        float torque_strength = 0.0f;
        float sas_damping = 0.0f;

        if (player_orbiter && player_orbiter->rails.active())
        {
            Entity *player_ent = context.world.entities().find(player_orbiter->entity);

            if (player_ent)
            {
                if (auto *sc_comp = player_ent->get_component<ShipController>())
                {
                    thrust_force_N = sc_comp->thrust_force();
                    torque_strength = sc_comp->torque_strength();
                    sas_damping = sc_comp->sas_damping();
                }
            }

            const ShipKeybinds *ship_keybinds = context.keybinds ? &context.keybinds->ship : nullptr;
            ThrustInput input = ShipController::read_input(ctx.input, ship_keybinds, ui_capture_keyboard,
                                                           player_orbiter->rails.sas_toggle_prev_down);
            if (input.sas_toggled)
            {
                player_orbiter->rails.sas_enabled = !player_orbiter->rails.sas_enabled;
                if (player_ent)
                {
                    if (auto *sc_comp = player_ent->get_component<ShipController>())
                    {
                        sc_comp->set_sas_enabled(player_orbiter->rails.sas_enabled);
                    }
                }
            }

            _rails_last_thrust_dir_local = input.local_thrust_dir;
            _rails_last_torque_dir_local = input.local_torque_dir;

            glm::vec3 world_torque_dir(0.0f);
            if (glm::length(input.local_torque_dir) > 0.0f)
            {
                world_torque_dir = player_orbiter->rails.rotation * input.local_torque_dir;
            }

            update_rails_rotation(player_orbiter->rails,
                                  world_torque_dir,
                                  torque_strength,
                                  sas_damping,
                                  player_orbiter->rails.sas_enabled,
                                  dt_s);

            const bool has_thrust = glm::length(input.local_thrust_dir) > 0.0f;
            if (has_thrust && thrust_force_N > 0.0f)
            {
                orbitsim::Spacecraft *sc = orbit.scenario_owner()->sim.spacecraft_by_id(player_orbiter->rails.sc_id);
                if (sc)
                {
                    const glm::dvec3 dir_world = glm::dvec3(player_orbiter->rails.rotation * input.local_thrust_dir);
                    const double mass_kg = std::max(1.0, sc->mass_kg());
                    const double dv = (static_cast<double>(thrust_force_N) / mass_kg) * dt_s;
                    const glm::dvec3 dv_world = dir_world * dv;
                    sc->state.velocity_mps += dv_world;
                    _rails_thrust_applied_this_tick = true;

                    for (auto &follower : orbit.orbiters())
                    {
                        if (!follower.formation_hold_enabled || follower.is_player)
                        {
                            continue;
                        }
                        if (follower.formation_leader_name != player_orbiter->name)
                        {
                            continue;
                        }
                        if (!follower.rails.active())
                        {
                            continue;
                        }
                        if (orbitsim::Spacecraft *fsc = orbit.scenario_owner()->sim.spacecraft_by_id(follower.rails.sc_id))
                        {
                            fsc->state.velocity_mps += dv_world;
                        }
                    }
                }
            }
        }

        update_formation_hold(context, dt_s);

        step_orbitsim_with_runtime_spacecraft_mode(context, dt_s);
        sync_celestial_render_entities(context, ctx);

        for (auto &orbiter : orbit.orbiters())
        {
            if (!orbiter.rails.active() || !orbiter.entity.is_valid())
            {
                continue;
            }

            if (!orbiter.is_player)
            {
                update_rails_rotation(orbiter.rails,
                                      glm::vec3(0.0f),
                                      0.0f,
                                      0.0f,
                                      false,
                                      dt_s);
            }

            const orbitsim::Spacecraft *sc = orbit.scenario_owner()->sim.spacecraft_by_id(orbiter.rails.sc_id);
            if (!sc)
            {
                continue;
            }

            const WorldVec3 body_pos_world = context.scenario_config.system_center +
                    WorldVec3(sc->state.position_m - ref_sim->state.position_m);

            Entity *ent = context.world.entities().find(orbiter.entity);
            if (!ent)
            {
                continue;
            }

            const WorldVec3 entity_pos_world =
                    OrbiterPhysicsBridge::entity_world_from_body_world(*ent, body_pos_world, orbiter.rails.rotation);

            if (ent->uses_interpolation())
            {
                ent->interpolation().store_current_as_previous();
                ent->interpolation().curr_position = entity_pos_world;
                ent->interpolation().curr_rotation = orbiter.rails.rotation;
            }

            ent->set_position_world(entity_pos_world);
            ent->set_rotation(orbiter.rails.rotation);
        }
    }

    GameplayOrbitalContextBuilder::GameplayOrbitalContextBuilder(GameplayOrbitalContextInputs inputs)
        : _inputs(std::move(inputs))
    {
    }

    OrbitalPhysicsSystem::Context GameplayOrbitalContextBuilder::build() const
    {
        GameplayOrbitalContextInputs inputs = _inputs;
        return OrbitalPhysicsSystem::Context{
            .renderer = inputs.renderer,
            .world = inputs.world,
            .orbit = inputs.orbit,
            .physics = inputs.physics,
            .physics_context = inputs.physics_context,
            .scenario_config = inputs.scenario_config,
            .spacecraft_gravity_mode = inputs.spacecraft_gravity_mode,
            .soi_switch_options = inputs.soi_switch_options,
            .kepler_propagation = inputs.kepler_propagation,
            .soi_kepler_max_step_s = inputs.soi_kepler_max_step_s,
            .keybinds = inputs.keybinds,
            .orbiter_world_state_sampler =
                    [inputs](const OrbiterInfo &sample_orbiter,
                             WorldVec3 &out_pos_world,
                             glm::dvec3 &out_vel_world,
                             glm::vec3 &out_vel_local) {
                        return OrbiterWorldStateProvider(OrbiterWorldStateProvider::Context{
                                .orbit = inputs.orbit,
                                .world = inputs.world,
                                .physics = inputs.physics,
                                .physics_context = inputs.physics_context,
                                .scenario_config = inputs.scenario_config,
                        }).get_orbiter_world_state(
                                sample_orbiter,
                                out_pos_world,
                                out_vel_world,
                                out_vel_local);
                    },
            .ui_capture_keyboard = std::move(inputs.ui_capture_keyboard),
            .mark_prediction_dirty = std::move(inputs.mark_prediction_dirty),
        };
    }
} // namespace Game
