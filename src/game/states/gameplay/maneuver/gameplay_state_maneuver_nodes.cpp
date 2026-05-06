#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/maneuver/maneuver_prediction_bridge.h"
#include "game/states/gameplay/prediction/gameplay_prediction_adapter.h"

namespace Game
{
    double GameplayState::current_sim_time_s() const
    {
        return _orbit.scenario_owner() ? _orbit.scenario_owner()->sim.time_s() : _fixed_time_s;
    }

    ManeuverPredictionBridgeContext GameplayState::build_maneuver_prediction_context()
    {
        return ManeuverPredictionBridgeContext{
            .maneuver = _maneuver,
            .prediction = *_prediction,
            .prediction_access = build_prediction_access(),
            .orbit = _orbit,
            .orbital_physics = _orbital_physics,
            .world = _world,
            .apply_maneuver_command = [this](const ManeuverCommand &command) {
                return apply_maneuver_command(command);
            },
            .current_sim_time_s = [this]() {
                return current_sim_time_s();
            },
        };
    }

    ManeuverCommandResult GameplayState::apply_maneuver_command(const ManeuverCommand &command)
    {
        ManeuverCommandResult result = _maneuver.apply_command(command);
        if (!result.applied)
        {
            return result;
        }

        if (result.clear_prediction_artifacts)
        {
            GameplayPredictionAdapter(build_prediction_access()).clear_maneuver_prediction_artifacts();
        }
        if (result.prediction_dirty)
        {
            GameplayPredictionAdapter(build_prediction_access()).mark_maneuver_plan_dirty();
        }
        else if (result.plan_changed)
        {
            const uint64_t revision = _maneuver.increment_revision();
            _prediction->invalidate_maneuver_plan_revision(revision);
        }

        return result;
    }
} // namespace Game
