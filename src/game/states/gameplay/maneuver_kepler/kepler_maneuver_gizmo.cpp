#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_gizmo.h"

#include "core/engine.h"
#include "core/input/input_system.h"
#include "core/render_viewport.h"
#include "game/state/game_state.h"
#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_gizmo_controller.h"
#include "game/states/gameplay/maneuver_kepler/kepler_maneuver_system.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Game::KeplerManeuverGizmo
{
    namespace
    {
        constexpr uint32_t rgba_u32(const uint32_t r, const uint32_t g, const uint32_t b, const uint32_t a)
        {
            return (a << 24u) | (b << 16u) | (g << 8u) | r;
        }

        double safe_length(const glm::dvec3 &v)
        {
            const double len2 = glm::dot(v, v);
            if (!(len2 > 0.0) || !std::isfinite(len2))
            {
                return 0.0;
            }
            return std::sqrt(len2);
        }

        ImVec2 to_imvec2(const glm::vec2 &p)
        {
            return ImVec2(p.x, p.y);
        }

        void clear_transient_interaction(KeplerManeuverInteraction &interaction)
        {
            const bool suppress_orbit_pick = interaction.suppress_orbit_pick_until_left_release;
            interaction = {};
            interaction.suppress_orbit_pick_until_left_release = suppress_orbit_pick;
        }

        bool compute_camera_ray(const GameStateContext &ctx,
                                const glm::vec2 &window_pos,
                                KeplerManeuverGizmoController::CameraRay &out_ray)
        {
            if (!ctx.renderer || !ctx.renderer->_sceneManager)
            {
                return false;
            }

            render::RenderViewportMetrics metrics{};
            glm::vec2 logical_pos{};
            if (!render::window_to_logical_render_pixels(*ctx.renderer, window_pos, logical_pos, &metrics))
            {
                return false;
            }

            const double width = static_cast<double>(metrics.logical_extent.width);
            const double height = static_cast<double>(metrics.logical_extent.height);
            if (!(width > 0.0) || !(height > 0.0))
            {
                return false;
            }

            const double ndc_x = (2.0 * static_cast<double>(logical_pos.x) / width) - 1.0;
            const double ndc_y = 1.0 - (2.0 * static_cast<double>(logical_pos.y) / height);

            const Camera &cam = ctx.renderer->_sceneManager->getMainCamera();
            const double fov_rad = glm::radians(static_cast<double>(cam.fovDegrees));
            const double tan_half_fov = std::tan(fov_rad * 0.5);
            const double aspect = width / height;

            glm::dvec3 dir_camera(ndc_x * aspect * tan_half_fov, ndc_y * tan_half_fov, -1.0);
            const double dir_len2 = glm::dot(dir_camera, dir_camera);
            if (!(dir_len2 > 0.0) || !std::isfinite(dir_len2))
            {
                return false;
            }
            dir_camera /= std::sqrt(dir_len2);

            const glm::mat4 cam_rot = cam.getRotationMatrix();
            const glm::vec3 dir_camera_f(static_cast<float>(dir_camera.x),
                                         static_cast<float>(dir_camera.y),
                                         static_cast<float>(dir_camera.z));
            glm::vec3 dir_world_f = glm::vec3(cam_rot * glm::vec4(dir_camera_f, 0.0f));
            const float dir_world_len2 = glm::dot(dir_world_f, dir_world_f);
            if (!(dir_world_len2 > 0.0f) || !std::isfinite(dir_world_len2))
            {
                return false;
            }
            dir_world_f = glm::normalize(dir_world_f);

            out_ray.camera_world = WorldVec3(cam.position_world);
            out_ray.ray_origin_local = glm::dvec3(0.0, 0.0, 0.0);
            out_ray.ray_dir_local = glm::dvec3(dir_world_f);
            return true;
        }

        void draw_markers(const KeplerManeuverSystem &maneuver,
                          ImDrawList *draw_list,
                          const std::vector<KeplerManeuverHubMarker> &hubs,
                          const std::vector<KeplerManeuverAxisMarker> &axes,
                          const std::vector<KeplerManeuverDeleteMarker> &deletes,
                          const KeplerManeuverGizmoHover &hover,
                          const float hub_hit_px,
                          const float axis_hit_px,
                          const float delete_hit_px)
        {
            if (!draw_list)
            {
                return;
            }

            constexpr ImU32 kHubFill = IM_COL32(235, 235, 245, 215);
            constexpr ImU32 kHubHover = IM_COL32(255, 255, 255, 245);
            constexpr ImU32 kHubSelectedFill = IM_COL32(255, 224, 92, 245);
            constexpr ImU32 kHubRing = IM_COL32(255, 255, 255, 170);
            constexpr ImU32 kHubSelectedRing = IM_COL32(255, 244, 160, 245);
            constexpr ImU32 kAxisHover = IM_COL32(255, 244, 190, 255);
            constexpr ImU32 kAxisActive = IM_COL32(255, 255, 255, 255);
            constexpr ImU32 kDeleteFill = IM_COL32(220, 58, 58, 230);
            constexpr ImU32 kDeleteHover = IM_COL32(255, 82, 82, 255);
            constexpr ImU32 kDeleteGlyph = IM_COL32(255, 255, 255, 245);

            for (const KeplerManeuverHubMarker &hub : hubs)
            {
                const bool selected = hub.node_id == maneuver.plan().selected_node_id;
                const bool hovered = hover.state == KeplerManeuverInteraction::State::HoverHub &&
                                     hover.node_id == hub.node_id;
                const float radius = selected ? hub_hit_px * 0.62f : hub_hit_px * 0.52f;
                draw_list->AddCircleFilled(to_imvec2(hub.screen),
                                           radius,
                                           selected ? kHubSelectedFill : (hovered ? kHubHover : kHubFill),
                                           24);
                draw_list->AddCircle(to_imvec2(hub.screen),
                                     radius + 2.0f,
                                     selected ? kHubSelectedRing : kHubRing,
                                     24,
                                     selected ? 2.0f : 1.5f);
            }

            const KeplerManeuverInteraction &interaction = maneuver.interaction();
            for (const KeplerManeuverAxisMarker &axis : axes)
            {
                const bool hovered = hover.state == KeplerManeuverInteraction::State::HoverAxis &&
                                     hover.node_id == axis.node_id &&
                                     hover.axis == axis.axis;
                const bool active = interaction.state == KeplerManeuverInteraction::State::DragAxis &&
                                    interaction.node_id == axis.node_id &&
                                    interaction.axis == axis.axis;
                const ImU32 color = active ? kAxisActive : (hovered ? kAxisHover : static_cast<ImU32>(axis.base_color));
                const float thickness = active ? 3.3f : (hovered ? 2.8f : 2.2f);
                const float handle_radius = active ? axis_hit_px * 0.96f
                                                   : (hovered ? axis_hit_px * 0.88f : axis_hit_px * 0.76f);

                draw_list->AddLine(to_imvec2(axis.hub_screen), to_imvec2(axis.handle_screen), color, thickness);
                draw_list->AddCircleFilled(to_imvec2(axis.handle_screen), handle_radius, color, 24);
                draw_list->AddText(ImVec2(axis.handle_screen.x + handle_radius + 3.0f,
                                          axis.handle_screen.y - handle_radius),
                                   color,
                                   axis.label);
            }

            for (const KeplerManeuverDeleteMarker &marker : deletes)
            {
                const bool hovered = hover.state == KeplerManeuverInteraction::State::HoverDelete &&
                                     hover.node_id == marker.node_id;
                const float radius = hovered ? delete_hit_px * 0.96f : delete_hit_px * 0.82f;
                const ImVec2 center = to_imvec2(marker.screen);
                draw_list->AddCircleFilled(center, radius, hovered ? kDeleteHover : kDeleteFill, 20);
                const float arm = radius * 0.42f;
                draw_list->AddLine(ImVec2(center.x - arm, center.y - arm),
                                   ImVec2(center.x + arm, center.y + arm),
                                   kDeleteGlyph,
                                   2.0f);
                draw_list->AddLine(ImVec2(center.x + arm, center.y - arm),
                                   ImVec2(center.x - arm, center.y + arm),
                                   kDeleteGlyph,
                                   2.0f);
            }
        }

        void draw_hover_tooltip(const KeplerManeuverSystem &maneuver,
                                const KeplerManeuverGizmoHover &hover)
        {
            if (hover.node_id < 0 ||
                hover.state == KeplerManeuverInteraction::State::Idle ||
                hover.state == KeplerManeuverInteraction::State::DragAxis)
            {
                return;
            }

            const KeplerManeuverEditorNode *node = maneuver.plan().find_node(hover.node_id);
            if (!node)
            {
                return;
            }

            ImGui::BeginTooltip();
            ImGui::Text("Node %d", node->id);
            if (hover.state == KeplerManeuverInteraction::State::HoverDelete)
            {
                ImGui::TextUnformatted("Delete");
            }
            else if (hover.state == KeplerManeuverInteraction::State::HoverAxis)
            {
                ImGui::Text("Axis: %s", axis_label(hover.axis));
                ImGui::Text("DV RTN: %.3f, %.3f, %.3f m/s",
                            node->dv_rtn_mps.x,
                            node->dv_rtn_mps.y,
                            node->dv_rtn_mps.z);
                ImGui::TextUnformatted("Shift x0.1 / Ctrl x10");
            }
            else
            {
                ImGui::Text("Time: %.3f s", node->time_s);
            }
            ImGui::EndTooltip();
        }

        KeplerManeuverCommandResult apply_command(KeplerManeuverGizmoDrawContext &context,
                                                  const KeplerManeuverCommand &command)
        {
            return context.apply_command ? context.apply_command(command) : KeplerManeuverCommandResult{};
        }
    } // namespace

    const char *axis_label(const KeplerManeuverHandleAxis axis)
    {
        switch (axis)
        {
            case KeplerManeuverHandleAxis::TangentialPos:
                return "+T";
            case KeplerManeuverHandleAxis::TangentialNeg:
                return "-T";
            case KeplerManeuverHandleAxis::RadialPos:
                return "+R";
            case KeplerManeuverHandleAxis::RadialNeg:
                return "-R";
            case KeplerManeuverHandleAxis::NormalPos:
                return "+N";
            case KeplerManeuverHandleAxis::NormalNeg:
                return "-N";
            default:
                return "";
        }
    }

    uint32_t axis_color(const KeplerManeuverHandleAxis axis)
    {
        switch (axis)
        {
            case KeplerManeuverHandleAxis::TangentialPos:
            case KeplerManeuverHandleAxis::TangentialNeg:
                return rgba_u32(95u, 235u, 105u, 220u);
            case KeplerManeuverHandleAxis::RadialPos:
            case KeplerManeuverHandleAxis::RadialNeg:
                return rgba_u32(240u, 100u, 100u, 220u);
            case KeplerManeuverHandleAxis::NormalPos:
            case KeplerManeuverHandleAxis::NormalNeg:
                return rgba_u32(95u, 160u, 255u, 220u);
            default:
                return rgba_u32(170u, 170u, 170u, 180u);
        }
    }

    bool is_axis(const KeplerManeuverHandleAxis axis)
    {
        return axis == KeplerManeuverHandleAxis::TangentialPos || axis == KeplerManeuverHandleAxis::TangentialNeg ||
               axis == KeplerManeuverHandleAxis::RadialPos || axis == KeplerManeuverHandleAxis::RadialNeg ||
               axis == KeplerManeuverHandleAxis::NormalPos || axis == KeplerManeuverHandleAxis::NormalNeg;
    }

    bool resolve_axis(const KeplerManeuverHandleAxis axis, const orbitsim::Vec3 &basis_r_world,
                      const orbitsim::Vec3 &basis_t_world, const orbitsim::Vec3 &basis_n_world,
                      orbitsim::Vec3 &out_axis_dir_world, int &out_component, double &out_sign)
    {
        out_axis_dir_world = orbitsim::Vec3{0.0, 1.0, 0.0};
        out_component = 1;
        out_sign = 1.0;

        switch (axis)
        {
            case KeplerManeuverHandleAxis::TangentialPos:
                out_axis_dir_world = basis_t_world;
                out_component = 1;
                out_sign = 1.0;
                break;
            case KeplerManeuverHandleAxis::TangentialNeg:
                out_axis_dir_world = -basis_t_world;
                out_component = 1;
                out_sign = -1.0;
                break;
            case KeplerManeuverHandleAxis::RadialPos:
                out_axis_dir_world = basis_r_world;
                out_component = 0;
                out_sign = 1.0;
                break;
            case KeplerManeuverHandleAxis::RadialNeg:
                out_axis_dir_world = -basis_r_world;
                out_component = 0;
                out_sign = -1.0;
                break;
            case KeplerManeuverHandleAxis::NormalPos:
                out_axis_dir_world = basis_n_world;
                out_component = 2;
                out_sign = 1.0;
                break;
            case KeplerManeuverHandleAxis::NormalNeg:
                out_axis_dir_world = -basis_n_world;
                out_component = 2;
                out_sign = -1.0;
                break;
            default:
                return false;
        }

        const double len = safe_length(out_axis_dir_world);
        if (!(len > 0.0) || !std::isfinite(len))
        {
            return false;
        }
        out_axis_dir_world /= len;
        return true;
    }

    bool is_occluded(const KeplerManeuverGizmoViewContext &view, const WorldVec3 &point_world)
    {
        if (!view.depth_occluder_valid)
        {
            return false;
        }

        const glm::dvec3 center_to_cam = view.camera_world - glm::dvec3(view.depth_occluder_center);
        const double center_to_cam_len2 = glm::dot(center_to_cam, center_to_cam);
        const double radius2 = view.depth_occluder_radius * view.depth_occluder_radius;
        if (!std::isfinite(center_to_cam_len2) || center_to_cam_len2 <= radius2)
        {
            return false;
        }

        const glm::dvec3 cam_to_point = glm::dvec3(point_world) - view.camera_world;
        const double point_dist = safe_length(cam_to_point);
        if (!std::isfinite(point_dist) || point_dist <= 1.0e-3)
        {
            return false;
        }

        const glm::dvec3 ray_dir = cam_to_point / point_dist;
        const glm::dvec3 oc = view.camera_world - glm::dvec3(view.depth_occluder_center);
        const double b = glm::dot(oc, ray_dir);
        const double c = glm::dot(oc, oc) - radius2;
        const double disc = b * b - c;
        if (!std::isfinite(disc) || disc <= 0.0)
        {
            return false;
        }

        const double t_near = -b - std::sqrt(disc);
        const double eps = std::max(1.0, view.depth_occluder_radius * 1.0e-6);
        return std::isfinite(t_near) && t_near > eps && t_near < (point_dist - eps);
    }

    bool project_point(const KeplerManeuverGizmoViewContext &view, const WorldVec3 &point_world, glm::vec2 &out_screen,
                       double &out_depth_m)
    {
        out_screen = glm::vec2(0.0f, 0.0f);
        out_depth_m = 0.0;

        const glm::dvec3 rel_world = glm::dvec3(point_world) - view.camera_world;
        const glm::dvec3 cam_space = view.world_to_cam * rel_world;
        const double depth_m = -cam_space.z;
        if (!std::isfinite(depth_m) || depth_m <= 1.0e-4)
        {
            return false;
        }

        const double ndc_x = cam_space.x / (depth_m * view.aspect * view.tan_half_fov);
        const double ndc_y = cam_space.y / (depth_m * view.tan_half_fov);
        if (!std::isfinite(ndc_x) || !std::isfinite(ndc_y))
        {
            return false;
        }
        if (ndc_x < -1.2 || ndc_x > 1.2 || ndc_y < -1.2 || ndc_y > 1.2)
        {
            return false;
        }

        const double u = (ndc_x + 1.0) * 0.5;
        const double v = (1.0 - ndc_y) * 0.5;
        if (!std::isfinite(u) || !std::isfinite(v))
        {
            return false;
        }

        const double swap_x =
            static_cast<double>(view.letterbox_rect.x) + u * static_cast<double>(view.letterbox_rect.width);
        const double swap_y =
            static_cast<double>(view.letterbox_rect.y) + v * static_cast<double>(view.letterbox_rect.height);
        const double draw_x = swap_x * view.draw_from_swap_x;
        const double draw_y = swap_y * view.draw_from_swap_y;
        const double win_x = draw_x * view.window_from_draw_x;
        const double win_y = draw_y * view.window_from_draw_y;
        if (!std::isfinite(win_x) || !std::isfinite(win_y))
        {
            return false;
        }

        out_screen = glm::vec2(static_cast<float>(win_x), static_cast<float>(win_y));
        out_depth_m = depth_m;
        return true;
    }

    const KeplerManeuverNodeDisplayState *find_display_state(
        const std::span<const KeplerManeuverNodeDisplayState> display_states, const int node_id)
    {
        for (const KeplerManeuverNodeDisplayState &state : display_states)
        {
            if (state.node_id == node_id)
            {
                return &state;
            }
        }
        return nullptr;
    }

    const KeplerManeuverNodeDisplaySnapshot *find_display_snapshot(
        const std::span<const KeplerManeuverNodeDisplaySnapshot> snapshots, const int node_id)
    {
        for (const KeplerManeuverNodeDisplaySnapshot &snapshot : snapshots)
        {
            if (snapshot.node_id == node_id)
            {
                return &snapshot;
            }
        }
        return nullptr;
    }

    KeplerManeuverGizmoHover find_hover(const std::span<const KeplerManeuverHubMarker> hubs,
                                        const std::span<const KeplerManeuverAxisMarker> axes,
                                        const std::span<const KeplerManeuverDeleteMarker> deletes,
                                        const glm::vec2 &mouse_pos, const float hub_hit_px2, const float axis_hit_px2,
                                        const float delete_hit_px2)
    {
        KeplerManeuverGizmoHover hover{};

        float best_axis_d2 = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < axes.size(); ++i)
        {
            const glm::vec2 d = axes[i].handle_screen - mouse_pos;
            const float d2 = glm::dot(d, d);
            if (d2 <= axis_hit_px2 && d2 < best_axis_d2)
            {
                best_axis_d2 = d2;
                hover.axis_index = static_cast<int>(i);
            }
        }
        if (hover.axis_index >= 0)
        {
            const KeplerManeuverAxisMarker &marker = axes[static_cast<std::size_t>(hover.axis_index)];
            hover.node_id = marker.node_id;
            hover.axis = marker.axis;
            hover.state = KeplerManeuverInteraction::State::HoverAxis;
            return hover;
        }

        float best_delete_d2 = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < deletes.size(); ++i)
        {
            const glm::vec2 d = deletes[i].screen - mouse_pos;
            const float d2 = glm::dot(d, d);
            if (d2 <= delete_hit_px2 && d2 < best_delete_d2)
            {
                best_delete_d2 = d2;
                hover.delete_index = static_cast<int>(i);
            }
        }
        if (hover.delete_index >= 0)
        {
            const KeplerManeuverDeleteMarker &marker = deletes[static_cast<std::size_t>(hover.delete_index)];
            hover.node_id = marker.node_id;
            hover.state = KeplerManeuverInteraction::State::HoverDelete;
            return hover;
        }

        float best_hub_d2 = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < hubs.size(); ++i)
        {
            const glm::vec2 d = hubs[i].screen - mouse_pos;
            const float d2 = glm::dot(d, d);
            if (d2 <= hub_hit_px2 && d2 < best_hub_d2)
            {
                best_hub_d2 = d2;
                hover.hub_index = static_cast<int>(i);
            }
        }
        if (hover.hub_index >= 0)
        {
            const KeplerManeuverHubMarker &marker = hubs[static_cast<std::size_t>(hover.hub_index)];
            hover.node_id = marker.node_id;
            hover.axis = KeplerManeuverHandleAxis::Hub;
            hover.state = KeplerManeuverInteraction::State::HoverHub;
        }

        return hover;
    }

    void draw(KeplerManeuverGizmoDrawContext &context)
    {
        GameStateContext &ctx = context.ctx;
        KeplerManeuverSystem &maneuver = context.maneuver;
        KeplerManeuverInteraction &interaction = maneuver.interaction();
        const KeplerManeuverPlanState &plan = maneuver.plan();

        if (!ctx.input || !ctx.renderer)
        {
            return;
        }

        const ImGuiIO &io = ImGui::GetIO();
        const bool keyboard_available = !io.WantCaptureKeyboard && !ImGui::IsAnyItemActive();
        if (keyboard_available &&
            interaction.state != KeplerManeuverInteraction::State::DragAxis &&
            plan.selected_node_id >= 0 &&
            (ctx.input->key_pressed(Key::Backspace) || ctx.input->key_pressed(Key::Delete)))
        {
            (void) apply_command(context, KeplerManeuverCommand::remove_node(plan.selected_node_id));
            return;
        }

        if (plan.nodes.empty())
        {
            return;
        }

        std::span<const KeplerManeuverNodeDisplayState> display_states = maneuver.node_display_states();
        if (display_states.empty())
        {
            return;
        }

        const float font_size = std::max(10.0f, ImGui::GetFontSize());
        const float overlay_size_px = std::max(12.0f, font_size * 1.05f);
        const float hub_hit_px = std::max(10.0f, overlay_size_px * 0.58f);
        const float axis_hit_px = std::max(9.0f, overlay_size_px * 0.50f);
        const float delete_hit_px = std::max(6.0f, overlay_size_px * 0.36f);

        std::vector<KeplerManeuverHubMarker> hubs{};
        std::vector<KeplerManeuverAxisMarker> axes{};
        std::vector<KeplerManeuverDeleteMarker> deletes{};
        KeplerManeuverGizmoController::build_markers(plan,
                                                     display_states,
                                                     interaction,
                                                     context.view,
                                                     overlay_size_px,
                                                     hubs,
                                                     axes,
                                                     deletes);

        const glm::vec2 mouse_pos(io.MousePos.x, io.MousePos.y);
        KeplerManeuverGizmoHover hover =
                find_hover(hubs,
                           axes,
                           deletes,
                           mouse_pos,
                           hub_hit_px * hub_hit_px,
                           axis_hit_px * axis_hit_px,
                           delete_hit_px * delete_hit_px);

        const bool can_capture_click = !io.WantCaptureMouse &&
                                       !ImGui::IsAnyItemActive() &&
                                       !ImGui::IsAnyItemHovered();

        if (interaction.state != KeplerManeuverInteraction::State::DragAxis)
        {
            if (hover.state == KeplerManeuverInteraction::State::HoverHub ||
                hover.state == KeplerManeuverInteraction::State::HoverAxis ||
                hover.state == KeplerManeuverInteraction::State::HoverDelete)
            {
                const bool suppress_orbit_pick = interaction.suppress_orbit_pick_until_left_release;
                interaction.state = hover.state;
                interaction.node_id = hover.node_id;
                interaction.axis = hover.axis;
                interaction.suppress_orbit_pick_until_left_release = suppress_orbit_pick;
            }
            else
            {
                clear_transient_interaction(interaction);
            }
        }

        bool rebuild_markers = false;
        if (ctx.input->mouse_pressed(MouseButton::Left) && can_capture_click)
        {
            if (hover.state == KeplerManeuverInteraction::State::HoverDelete && hover.node_id >= 0)
            {
                interaction.suppress_orbit_pick_until_left_release = true;
                (void) apply_command(context, KeplerManeuverCommand::remove_node(hover.node_id));
                return;
            }

            if (hover.state == KeplerManeuverInteraction::State::HoverAxis && hover.node_id >= 0)
            {
                interaction.suppress_orbit_pick_until_left_release = true;
                (void) apply_command(context, KeplerManeuverCommand::select_node(hover.node_id));

                KeplerManeuverGizmoController::CameraRay ray{};
                if (compute_camera_ray(ctx, mouse_pos, ray) &&
                    KeplerManeuverGizmoController::begin_axis_drag(plan,
                                                                   maneuver.node_display_states(),
                                                                   interaction,
                                                                   hover.node_id,
                                                                   hover.axis,
                                                                   ray,
                                                                   mouse_pos))
                {
                    interaction.suppress_orbit_pick_until_left_release = true;
                    rebuild_markers = true;
                }
            }
            else if (hover.state == KeplerManeuverInteraction::State::HoverHub && hover.node_id >= 0)
            {
                interaction.suppress_orbit_pick_until_left_release = true;
                (void) apply_command(context, KeplerManeuverCommand::select_node(hover.node_id));
                rebuild_markers = true;
            }
        }

        if (interaction.state == KeplerManeuverInteraction::State::DragAxis)
        {
            const KeplerManeuverEditorNode *node = maneuver.plan().find_node(interaction.node_id);
            const KeplerManeuverNodeDisplayState *display =
                    find_display_state(maneuver.node_display_states(), interaction.node_id);
            if (!node || !display || !display->valid)
            {
                clear_transient_interaction(interaction);
                return;
            }

            if (!ctx.input->mouse_down(MouseButton::Left))
            {
                const bool changed = interaction.applied_delta;
                if (hover.state == KeplerManeuverInteraction::State::HoverAxis)
                {
                    const bool suppress_orbit_pick = interaction.suppress_orbit_pick_until_left_release;
                    interaction.state = hover.state;
                    interaction.node_id = hover.node_id;
                    interaction.axis = hover.axis;
                    interaction.applied_delta = false;
                    interaction.suppress_orbit_pick_until_left_release = suppress_orbit_pick;
                }
                else
                {
                    clear_transient_interaction(interaction);
                }

                if (changed)
                {
                    (void) apply_command(context, KeplerManeuverCommand::mark_plan_dirty());
                    return;
                }
                rebuild_markers = true;
            }
            else
            {
                KeplerManeuverGizmoController::CameraRay ray{};
                if (compute_camera_ray(ctx, mouse_pos, ray))
                {
                    const InputModifiers mods = ctx.input->modifiers();
                    const KeplerManeuverGizmoController::DragUpdateResult update =
                            KeplerManeuverGizmoController::update_axis_drag(
                                    interaction,
                                    *node,
                                    *display,
                                    mouse_pos,
                                    ray,
                                    std::max(io.MouseDragThreshold, 2.0f),
                                    1.0,
                                    mods.ctrl,
                                    mods.shift);
                    if (update.clear_interaction)
                    {
                        clear_transient_interaction(interaction);
                        return;
                    }
                    if (update.changed)
                    {
                        (void) apply_command(context,
                                             KeplerManeuverCommand::set_node_dv(node->id,
                                                                                update.dv_rtn_mps));
                        interaction.applied_delta = true;
                        rebuild_markers = true;
                    }
                }
            }
        }

        if (rebuild_markers)
        {
            display_states = maneuver.node_display_states();
            hubs.clear();
            axes.clear();
            deletes.clear();
            KeplerManeuverGizmoController::build_markers(plan,
                                                         display_states,
                                                         interaction,
                                                         context.view,
                                                         overlay_size_px,
                                                         hubs,
                                                         axes,
                                                         deletes);
            hover = find_hover(hubs,
                               axes,
                               deletes,
                               mouse_pos,
                               hub_hit_px * hub_hit_px,
                               axis_hit_px * axis_hit_px,
                               delete_hit_px * delete_hit_px);
        }

        ImDrawList *draw_list = ImGui::GetForegroundDrawList();
        draw_markers(maneuver,
                     draw_list,
                     hubs,
                     axes,
                     deletes,
                     hover,
                     hub_hit_px,
                     axis_hit_px,
                     delete_hit_px);

        if (interaction.state != KeplerManeuverInteraction::State::DragAxis)
        {
            draw_hover_tooltip(maneuver, hover);
        }
    }
} // namespace Game::KeplerManeuverGizmo
