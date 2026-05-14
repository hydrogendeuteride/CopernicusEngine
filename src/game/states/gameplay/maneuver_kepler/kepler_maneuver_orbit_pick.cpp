#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_orbit_pick.h"

#include <cmath>

namespace Game
{
    namespace
    {
        constexpr std::string_view kKeplerOrbitBaseOwner = "KeplerOrbit/Base";
        constexpr std::string_view kKeplerOrbitPlannedOwner = "KeplerOrbit/Planned";

        bool match_owner_prefix(const std::string_view owner_name,
                                const std::string_view prefix,
                                std::string_view *out_track_label)
        {
            if (out_track_label)
            {
                *out_track_label = {};
            }

            if (owner_name == prefix)
            {
                return true;
            }

            if (owner_name.size() <= prefix.size() + 1u ||
                owner_name.substr(0u, prefix.size()) != prefix ||
                owner_name[prefix.size()] != '/')
            {
                return false;
            }

            if (out_track_label)
            {
                *out_track_label = owner_name.substr(prefix.size() + 1u);
            }
            return true;
        }

        const KeplerPredictionState::Track *find_active_player_track(
                const KeplerPredictionState &prediction)
        {
            for (const KeplerPredictionState::Track &track : prediction.tracks)
            {
                if (track.valid && track.active_player && !track.celestial)
                {
                    return &track;
                }
            }
            return nullptr;
        }

        bool payload_track_matches(const Picking::LinePickPayload &payload,
                                   const KeplerPredictionState::Track &track)
        {
            return payload.type == KeplerManeuverPick::kOrbitLinePickPayloadType &&
                   payload.primary_id != 0u &&
                   track.entity.is_valid() &&
                   payload.primary_id == static_cast<uint64_t>(track.entity.value);
        }
    } // namespace

    namespace KeplerManeuverPick
    {
        Picking::LinePickPayload make_payload(const KeplerManeuverOrbitPickRole role,
                                              const EntityId track_entity)
        {
            if (role == KeplerManeuverOrbitPickRole::None || !track_entity.is_valid())
            {
                return {};
            }

            return Picking::LinePickPayload{
                    .type = kOrbitLinePickPayloadType,
                    .role = static_cast<uint32_t>(role),
                    .primary_id = static_cast<uint64_t>(track_entity.value),
                    .secondary_id = 0u,
            };
        }

        KeplerManeuverOrbitPickRole payload_role(const Picking::LinePickPayload &payload)
        {
            if (payload.type != kOrbitLinePickPayloadType)
            {
                return KeplerManeuverOrbitPickRole::None;
            }

            const auto role = static_cast<KeplerManeuverOrbitPickRole>(payload.role);
            switch (role)
            {
                case KeplerManeuverOrbitPickRole::Base:
                case KeplerManeuverOrbitPickRole::Planned:
                    return role;
                case KeplerManeuverOrbitPickRole::None:
                    break;
            }
            return KeplerManeuverOrbitPickRole::None;
        }

        KeplerManeuverOrbitPickRole parse_owner(
                const std::string_view owner_name,
                std::string_view *out_track_label)
        {
            if (match_owner_prefix(owner_name, kKeplerOrbitBaseOwner, out_track_label))
            {
                return KeplerManeuverOrbitPickRole::Base;
            }
            if (match_owner_prefix(owner_name, kKeplerOrbitPlannedOwner, out_track_label))
            {
                return KeplerManeuverOrbitPickRole::Planned;
            }
            if (out_track_label)
            {
                *out_track_label = {};
            }
            return KeplerManeuverOrbitPickRole::None;
        }

        bool can_create_node(
                const KeplerManeuverOrbitPickInfo &pick,
                const KeplerManeuverPlanState &plan,
                const KeplerPredictionState &prediction,
                const double current_time_s)
        {
            // Only future orbit-line picks can create nodes.
            if (!pick.valid ||
                !pick.line ||
                !std::isfinite(pick.time_s) ||
                !std::isfinite(current_time_s) ||
                pick.time_s <= current_time_s)
            {
                return false;
            }

            const KeplerPredictionState::Track *active_track = find_active_player_track(prediction);
            if (!active_track)
            {
                return false;
            }

            KeplerManeuverOrbitPickRole role = payload_role(pick.payload);
            bool track_matches = payload_track_matches(pick.payload, *active_track);
            std::string_view pick_track_label{};
            if (role == KeplerManeuverOrbitPickRole::None)
            {
                role = parse_owner(pick.owner_name, &pick_track_label);
                track_matches = pick_track_label == active_track->label;
            }
            if (role == KeplerManeuverOrbitPickRole::None)
            {
                return false;
            }

            // First node uses base orbit; later nodes use the planned chain.
            const bool plan_empty = plan.nodes.empty();
            if ((plan_empty && role != KeplerManeuverOrbitPickRole::Base) ||
                (!plan_empty && role != KeplerManeuverOrbitPickRole::Planned))
            {
                return false;
            }

            return track_matches;
        }

        KeplerManeuverEditorNode make_node(
                const KeplerManeuverOrbitPickInfo &pick)
        {
            KeplerManeuverEditorNode node{};
            node.time_s = pick.time_s;
            node.primary_body_id = orbitsim::kInvalidBodyId;
            node.primary_body_auto = true;
            node.dv_rtn_mps = orbitsim::Vec3{0.0, 0.0, 0.0};
            return node;
        }
    } // namespace KeplerManeuverPick
} // namespace Game
