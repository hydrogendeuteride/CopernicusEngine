#pragma once

#include "core/world.h"
#include "game/orbit/kepler/kepler_types.h"

#include <glm/vec2.hpp>

#include <cstdint>
#include <limits>
#include <vector>

namespace Game
{
    enum class KeplerManeuverNodeDisplaySource : uint8_t
    {
        None = 0,
        BaseArc,
        PlannedPreImpulseArc,
    };

    enum class KeplerManeuverNodeDisplayStatus : uint8_t
    {
        Unresolved = 0,
        Resolved,
        MissingActiveTrack,
        InvalidTrack,
        InvalidNode,
        OutsideArc,
        PropagationFailed,
        PrimaryUnavailable,
    };

    // Resolved world-space data used to draw and interact with one maneuver node.
    struct KeplerManeuverNodeDisplayState
    {
        int node_id{-1};
        bool valid{false};
        bool selected{false};
        KeplerManeuverNodeDisplaySource source{KeplerManeuverNodeDisplaySource::None};
        KeplerManeuverNodeDisplayStatus status{KeplerManeuverNodeDisplayStatus::Unresolved};
        double time_s{std::numeric_limits<double>::quiet_NaN()};
        orbitsim::BodyId primary_body_id{orbitsim::kInvalidBodyId};
        WorldVec3 position_world{0.0, 0.0, 0.0};
        orbitsim::Vec3 basis_r_world{1.0, 0.0, 0.0};
        orbitsim::Vec3 basis_t_world{0.0, 1.0, 0.0};
        orbitsim::Vec3 basis_n_world{0.0, 0.0, 1.0};
        orbitsim::Vec3 burn_direction_world{0.0, 1.0, 0.0};
        double total_dv_mps{0.0};
    };

    // User-authored maneuver node stored by the editor before prediction normalization.
    struct KeplerManeuverEditorNode
    {
        int id{-1};
        double time_s{0.0};
        orbitsim::BodyId primary_body_id{orbitsim::kInvalidBodyId};
        bool primary_body_auto{true};
        orbitsim::Vec3 dv_rtn_mps{0.0, 0.0, 0.0};

        KeplerManeuverNode to_prediction_node() const;
    };

    // Mutable maneuver plan owned by the Kepler maneuver system.
    struct KeplerManeuverPlanState
    {
        std::vector<KeplerManeuverEditorNode> nodes{};
        int selected_node_id{-1};
        int next_node_id{0};

        KeplerManeuverEditorNode *find_node(int id);
        const KeplerManeuverEditorNode *find_node(int id) const;
        bool sort_by_time();
        std::vector<KeplerManeuverNode> to_prediction_nodes() const;
    };

    // Screen handle selected by the maneuver gizmo.
    enum class KeplerManeuverHandleAxis : uint8_t
    {
        None = 0,
        Hub,
        TangentialPos,
        TangentialNeg,
        RadialPos,
        RadialNeg,
        NormalPos,
        NormalNeg,
    };

    // Stable display data captured at drag start to avoid gizmo jumps while prediction updates.
    struct KeplerManeuverNodeDisplaySnapshot
    {
        int node_id{-1};
        WorldVec3 position_world{0.0, 0.0, 0.0};
        orbitsim::Vec3 basis_r_world{1.0, 0.0, 0.0};
        orbitsim::Vec3 basis_t_world{0.0, 1.0, 0.0};
        orbitsim::Vec3 basis_n_world{0.0, 0.0, 1.0};
    };

    // Transient hover and drag state for the Kepler maneuver gizmo.
    struct KeplerManeuverInteraction
    {
        enum class State
        {
            Idle = 0,
            HoverHub,
            HoverAxis,
            DragAxis,
            HoverDelete,
        };

        State state{State::Idle};
        int node_id{-1};
        KeplerManeuverHandleAxis axis{KeplerManeuverHandleAxis::None};
        orbitsim::Vec3 start_dv_rtn_mps{0.0, 0.0, 0.0};
        double start_axis_t_m{0.0};
        orbitsim::Vec3 drag_basis_r_world{1.0, 0.0, 0.0};
        orbitsim::Vec3 drag_basis_t_world{0.0, 1.0, 0.0};
        orbitsim::Vec3 drag_basis_n_world{0.0, 0.0, 1.0};
        glm::vec2 drag_start_mouse_window_pos{0.0f, 0.0f};
        glm::vec2 drag_last_sample_mouse_window_pos{0.0f, 0.0f};
        bool drag_threshold_passed{false};
        std::vector<KeplerManeuverNodeDisplaySnapshot> drag_display_snapshots{};
        bool applied_delta{false};
        bool suppress_orbit_pick_until_left_release{false};
    };
} // namespace Game
