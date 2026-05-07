#pragma once

#include "game/orbit/kepler/kepler_celestial_nbody.h"
#include "game/orbit/kepler/kepler_prediction_options.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_builder.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_draw.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_state.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_subject.h"

#include <span>
#include <vector>

namespace Game
{
    struct KeplerPredictionUpdateContext
    {
        const OrbitalRuntimeSystem *orbit{nullptr};
        const GameWorld *world{nullptr};
        const Physics::PhysicsWorld *physics{nullptr};
        const Physics::PhysicsContext *physics_context{nullptr};
        const ScenarioConfig *scenario_config{nullptr};
        bool enabled{true};
        double current_sim_time_s{0.0};
        double requested_horizon_s{0.0};
        orbitsim::BodyId fixed_primary_body_id{orbitsim::kInvalidBodyId};
        KeplerPredictionOptions options{};
        KeplerOrbitTessellationOptions tessellation{};
        std::span<const KeplerManeuverNode> maneuver_nodes{};
        uint64_t maneuver_revision{0};
        double celestial_nbody_horizon_s{0.0};
        bool build_celestial_nbody_tracks{true};
    };

    class KeplerPredictionSystem
    {
    public:
        [[nodiscard]] KeplerPredictionState &state() { return _state; }
        [[nodiscard]] const KeplerPredictionState &state() const { return _state; }

        void reset();
        void mark_dirty();
        void update(const KeplerPredictionUpdateContext &context);
        void draw(const KeplerPredictionDrawContext &context) const;

    private:
        struct CelestialNBodyEphemerisCache
        {
            bool valid{false};
            const orbitsim::GameSimulation *simulation{nullptr};
            KeplerSharedCelestialEphemeris ephemeris{};
            KeplerBodyStateProvider body_state_provider{};
            double t0_s{0.0};
            double t_end_s{0.0};
            double required_horizon_s{0.0};
            double built_horizon_s{0.0};
            double gravitational_constant{0.0};
            double softening_length_m{0.0};
            KeplerCelestialNBodyEphemerisOptions ephemeris_options{};
            orbitsim::BodyId world_reference_body_id{orbitsim::kInvalidBodyId};
            std::vector<orbitsim::BodyId> body_ids{};
            std::vector<double> body_masses_kg{};
            KeplerOrbitStatus status{KeplerOrbitStatus::InvalidInput};
            orbitsim::AdaptiveEphemerisDiagnostics diagnostics{};
        };

        [[nodiscard]] const CelestialNBodyEphemerisCache &resolve_celestial_nbody_cache(
                const KeplerPredictionUpdateContext &context,
                const KeplerWorldFrame &world_frame);

        KeplerPredictionState _state{};
        CelestialNBodyEphemerisCache _celestial_nbody_cache{};
    };
} // namespace Game
