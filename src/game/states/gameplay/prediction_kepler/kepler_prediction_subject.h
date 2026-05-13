#pragma once

// Kepler prediction subject adapter.
// Gameplay/world state -> inertial Kepler inputs.

#include "core/world.h"
#include "game/entity.h"
#include "game/orbit/kepler/kepler_types.h"

#include <glm/glm.hpp>

#include <string>
#include <vector>

namespace Physics
{
    class PhysicsContext;
    class PhysicsWorld;
} // namespace Physics

namespace Game
{
    class GameWorld;
    class OrbitalRuntimeSystem;
    struct ScenarioConfig;

    // Input bundle for subject resolution.
    struct KeplerPredictionSubjectContext
    {
        const OrbitalRuntimeSystem *orbit{nullptr};
        const GameWorld *world{nullptr};
        const Physics::PhysicsWorld *physics{nullptr};
        const Physics::PhysicsContext *physics_context{nullptr};
        const ScenarioConfig *scenario_config{nullptr};
    };

    // Resolved prediction target.
    struct KeplerPredictionSubject
    {
        bool valid{false};
        bool celestial{false};
        bool active_player{false};
        EntityId entity{};
        orbitsim::BodyId body_id{orbitsim::kInvalidBodyId};
        std::string label{};
        glm::vec3 orbit_rgb{0.18f, 0.82f, 1.0f};
        WorldVec3 position_world{0.0, 0.0, 0.0};
        glm::dvec3 velocity_world_mps{0.0, 0.0, 0.0};
        orbitsim::State state_inertial{};
    };

    // Spacecraft/entity targets.
    std::vector<KeplerPredictionSubject> resolve_kepler_prediction_orbiter_subjects(
            const KeplerPredictionSubjectContext &context,
            const KeplerWorldFrame &world_frame);

    // Celestial body targets.
    std::vector<KeplerPredictionSubject> resolve_kepler_prediction_celestial_subjects(
            const KeplerPredictionSubjectContext &context);

    // World <-> inertial frame bridge.
    bool resolve_kepler_prediction_world_frame(
            const KeplerPredictionSubjectContext &context,
            KeplerWorldFrame &out_frame);

    // Current-state fallback body provider.
    KeplerBodyStateProvider make_current_kepler_body_state_provider(
            const OrbitalRuntimeSystem &orbit);
} // namespace Game
