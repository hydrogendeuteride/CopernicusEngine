#pragma once

#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_types.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_state.h"

#include <cstdint>
#include <string_view>

namespace Game
{
    enum class KeplerManeuverOrbitPickRole : uint8_t
    {
        None = 0,
        Base,
        Planned,
    };

    struct KeplerManeuverOrbitPickInfo
    {
        bool valid{false};
        bool line{false};
        std::string_view owner_name{};
        double time_s{0.0};
    };

    namespace KeplerManeuverPick
    {
        // Reads "KeplerOrbit/Base/<track>" or "KeplerOrbit/Planned/<track>".
        KeplerManeuverOrbitPickRole parse_owner(
                std::string_view owner_name,
                std::string_view *out_track_label = nullptr);

        // Allows the first node on base orbit, then later nodes on planned orbit.
        bool can_create_node(
                const KeplerManeuverOrbitPickInfo &pick,
                const KeplerManeuverPlanState &plan,
                const KeplerPredictionState &prediction,
                double current_time_s);

        // Creates a timed zero-dv node; gizmo editing supplies dv later.
        KeplerManeuverEditorNode make_node(
                const KeplerManeuverOrbitPickInfo &pick);
    } // namespace KeplerManeuverPick
} // namespace Game
