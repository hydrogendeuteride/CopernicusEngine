#include "game/states/gameplay/prediction_kepler/kepler_prediction_subject.h"

#include "game/game_world.h"
#include "game/states/gameplay/orbital/orbiter_state_bridge.h"
#include "game/states/gameplay/orbital/orbital_runtime_system.h"
#include "game/states/gameplay/scenario/scenario_config.h"

#include <utility>

namespace Game
{
    namespace
    {
        constexpr glm::vec3 kOrbiterPalette[] = {
                {1.00f, 0.25f, 0.25f},
                {0.25f, 0.52f, 1.00f},
                {0.62f, 0.65f, 0.70f},
                {0.28f, 0.88f, 0.38f},
        };
        constexpr glm::vec3 kDefaultCelestialColor{0.80f, 0.82f, 0.86f};

        glm::vec3 orbiter_color(const ScenarioConfig &config,
                                const OrbiterInfo &orbiter,
                                const std::size_t index)
        {
            for (const ScenarioConfig::OrbiterDef &def : config.orbiters)
            {
                if (def.name == orbiter.name && def.has_prediction_orbit_color)
                {
                    return def.prediction_orbit_color;
                }
            }
            return kOrbiterPalette[index % (sizeof(kOrbiterPalette) / sizeof(kOrbiterPalette[0]))];
        }

        glm::vec3 celestial_color(const ScenarioConfig &config,
                                  const CelestialBodyInfo &body)
        {
            for (const ScenarioConfig::CelestialDef &def : config.celestials)
            {
                if (def.name == body.name && def.has_prediction_orbit_color)
                {
                    return def.prediction_orbit_color;
                }
            }
            return kDefaultCelestialColor;
        }
    } // namespace

    bool resolve_kepler_prediction_world_frame(
            const KeplerPredictionSubjectContext &context,
            KeplerWorldFrame &out_frame)
    {
        out_frame = {};
        if (!context.orbit || !context.world || !context.scenario_config)
        {
            return false;
        }

        const OrbitalScenario *scenario = context.orbit->scenario();
        if (!scenario)
        {
            return false;
        }

        const CelestialBodyInfo *world_ref_info = scenario->world_reference_body();
        const orbitsim::MassiveBody *world_ref_sim = scenario->world_reference_sim_body();
        if (!world_ref_info || !world_ref_sim || world_ref_info->sim_id == orbitsim::kInvalidBodyId)
        {
            return false;
        }

        out_frame.world_reference_body_world = context.scenario_config->system_center;
        if (world_ref_info->render_entity.is_valid())
        {
            if (const Entity *entity = context.world->entities().find(world_ref_info->render_entity))
            {
                out_frame.world_reference_body_world = entity->position_world();
            }
        }

        out_frame.world_reference_body_id = world_ref_info->sim_id;
        out_frame.world_reference_state_inertial = world_ref_sim->state;
        return kepler_finite_vec3(out_frame.world_reference_state_inertial.position_m);
    }

    std::vector<KeplerPredictionSubject> resolve_kepler_prediction_orbiter_subjects(
            const KeplerPredictionSubjectContext &context,
            const KeplerWorldFrame &world_frame)
    {
        std::vector<KeplerPredictionSubject> out;
        if (!context.orbit || !context.world || !context.scenario_config)
        {
            return out;
        }

        const OrbitalScenario *scenario = context.orbit->scenario();
        const orbitsim::MassiveBody *world_ref_sim =
                scenario ? scenario->world_reference_sim_body() : nullptr;
        if (!scenario || !world_ref_sim)
        {
            return out;
        }

        const EntityId player_entity = context.orbit->player_entity();
        OrbiterWorldStateProvider state_provider(OrbiterWorldStateProvider::Context{
                    .orbit = *context.orbit,
                    .world = *context.world,
                    .physics = context.physics,
                    .physics_context = context.physics_context,
                    .scenario_config = *context.scenario_config,
                });

        out.reserve(context.orbit->orbiters().size());
        for (std::size_t i = 0; i < context.orbit->orbiters().size(); ++i)
        {
            const OrbiterInfo &orbiter = context.orbit->orbiters()[i];
            if (!orbiter.entity.is_valid())
            {
                continue;
            }

            WorldVec3 position_world{0.0, 0.0, 0.0};
            glm::dvec3 velocity_world_mps{0.0, 0.0, 0.0};
            glm::vec3 velocity_local_mps{0.0f, 0.0f, 0.0f};
            if (!state_provider.get_orbiter_world_state(orbiter,
                                                        position_world,
                                                        velocity_world_mps,
                                                        velocity_local_mps))
            {
                continue;
            }

            const glm::dvec3 position_rel_ref_m =
                    glm::dvec3(position_world - world_frame.world_reference_body_world);
            const glm::dvec3 position_inertial_m =
                    world_ref_sim->state.position_m + position_rel_ref_m;
            const glm::dvec3 velocity_inertial_mps =
                    world_ref_sim->state.velocity_mps + velocity_world_mps;
            if (!kepler_finite_vec3(position_inertial_m) ||
                !kepler_finite_vec3(velocity_inertial_mps))
            {
                continue;
            }

            KeplerPredictionSubject subject{};
            subject.valid = true;
            subject.entity = orbiter.entity;
            subject.label = orbiter.name;
            subject.active_player = orbiter.entity == player_entity;
            subject.orbit_rgb = orbiter_color(*context.scenario_config, orbiter, i);
            subject.position_world = position_world;
            subject.velocity_world_mps = velocity_world_mps;
            subject.state_inertial = orbitsim::make_state(position_inertial_m, velocity_inertial_mps);
            out.push_back(std::move(subject));
        }

        return out;
    }

    std::vector<KeplerPredictionSubject> resolve_kepler_prediction_celestial_subjects(
            const KeplerPredictionSubjectContext &context)
    {
        std::vector<KeplerPredictionSubject> out;
        if (!context.orbit || !context.scenario_config)
        {
            return out;
        }

        const OrbitalScenario *scenario = context.orbit->scenario();
        const CelestialBodyInfo *world_ref_info = scenario ? scenario->world_reference_body() : nullptr;
        if (!scenario || !world_ref_info)
        {
            return out;
        }

        out.reserve(scenario->bodies.size());
        for (const CelestialBodyInfo &body_info : scenario->bodies)
        {
            if (body_info.sim_id == world_ref_info->sim_id)
            {
                continue;
            }

            const orbitsim::MassiveBody *body = scenario->sim.body_by_id(body_info.sim_id);
            if (!body ||
                !kepler_finite_vec3(body->state.position_m) ||
                !kepler_finite_vec3(body->state.velocity_mps))
            {
                continue;
            }

            KeplerPredictionSubject subject{};
            subject.valid = true;
            subject.celestial = true;
            subject.body_id = body_info.sim_id;
            subject.label = body_info.name;
            subject.orbit_rgb = celestial_color(*context.scenario_config, body_info);
            subject.state_inertial = body->state;
            out.push_back(std::move(subject));
        }

        return out;
    }

    KeplerBodyStateProvider make_current_kepler_body_state_provider(const OrbitalRuntimeSystem &orbit)
    {
        KeplerBodyStateProvider provider{};
        const OrbitalScenario *scenario = orbit.scenario();
        provider.state_at = [scenario](const orbitsim::BodyId body_id,
                                       const double /*t_s*/,
                                       orbitsim::State &out_state) {
            if (!scenario)
            {
                return false;
            }

            const orbitsim::MassiveBody *body = scenario->sim.body_by_id(body_id);
            if (!body)
            {
                return false;
            }

            out_state = body->state;
            return kepler_finite_vec3(out_state.position_m);
        };
        return provider;
    }
} // namespace Game
