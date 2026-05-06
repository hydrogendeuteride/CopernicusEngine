#include "gameplay_state.h"
#include "orbit_helpers.h"
#include "game/orbit/nbody/tuning.h"
#include "game/states/gameplay/gameplay_settings.h"
#include "game/states/gameplay/maneuver_nbody/gameplay_state_maneuver_gizmo_helpers.h"
#include "game/states/gameplay/maneuver_nbody/maneuver_ui_controller.h"
#include "game/states/gameplay/prediction_nbody/gameplay_prediction_adapter.h"
#include "game/states/gameplay/prediction_nbody/prediction_host_context_builder.h"
#include "game/states/gameplay/scenario/scenario_loader.h"
#include "game/component/ship_controller.h"
#include "core/debug_draw/debug_draw.h"
#include "core/engine.h"
#include "core/game_api.h"
#include "core/orbit_plot/orbit_plot.h"
#include "core/util/logger.h"
#include "physics/physics_context.h"
#include "physics/physics_world.h"

#include "imgui.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <filesystem>
#include <string>

namespace Game
{
    namespace Gizmo = ManeuverGizmoHelpers;

    using detail::contact_event_type_name;

    namespace
    {
        const char *prediction_solver_status_label(const OrbitPredictionStatus status)
        {
            switch (status)
            {
                case OrbitPredictionStatus::None:
                    return "None";
                case OrbitPredictionStatus::Success:
                    return "Success";
                case OrbitPredictionStatus::InvalidInput:
                    return "Invalid input";
                case OrbitPredictionStatus::InvalidSubject:
                    return "Invalid subject";
                case OrbitPredictionStatus::InvalidSamplingSpec:
                    return "Invalid sampling spec";
                case OrbitPredictionStatus::EphemerisUnavailable:
                    return "Ephemeris unavailable";
                case OrbitPredictionStatus::TrajectorySegmentsUnavailable:
                    return "Segments unavailable";
                case OrbitPredictionStatus::TrajectorySamplesUnavailable:
                    return "Samples unavailable";
                case OrbitPredictionStatus::ContinuityFailed:
                    return "Continuity failed";
                case OrbitPredictionStatus::Cancelled:
                    return "Cancelled";
            }

            return "Unknown";
        }

        const char *prediction_derived_status_label(const PredictionDerivedStatus status)
        {
            switch (status)
            {
                case PredictionDerivedStatus::None:
                    return "None";
                case PredictionDerivedStatus::Success:
                    return "Success";
                case PredictionDerivedStatus::MissingSolverData:
                    return "Missing solver data";
                case PredictionDerivedStatus::MissingEphemeris:
                    return "Missing ephemeris";
                case PredictionDerivedStatus::FrameTransformFailed:
                    return "Frame transform failed";
                case PredictionDerivedStatus::FrameSamplesUnavailable:
                    return "Frame samples unavailable";
                case PredictionDerivedStatus::ContinuityFailed:
                    return "Continuity failed";
                case PredictionDerivedStatus::Cancelled:
                    return "Cancelled";
            }

            return "Unknown";
        }

        const char *prediction_solve_quality_label(const OrbitPredictionSolveQuality quality)
        {
            switch (quality)
            {
                case OrbitPredictionSolveQuality::Full:
                    return "Full";
                case OrbitPredictionSolveQuality::FastPreview:
                    return "FastPreview";
            }

            return "Unknown";
        }

        const char *prediction_preview_state_label(const PredictionPreviewRuntimeState state)
        {
            switch (state)
            {
                case PredictionPreviewRuntimeState::Idle:
                    return "Idle";
                case PredictionPreviewRuntimeState::EnterDrag:
                    return "EnterDrag";
                case PredictionPreviewRuntimeState::DragPreviewPending:
                    return "DragPreviewPending";
                case PredictionPreviewRuntimeState::PreviewStreaming:
                    return "PreviewStreaming";
                case PredictionPreviewRuntimeState::AwaitFullRefine:
                    return "AwaitFullRefine";
            }

            return "Unknown";
        }

        const char *gizmo_interaction_state_label(const ManeuverGizmoInteraction::State state)
        {
            switch (state)
            {
                case ManeuverGizmoInteraction::State::Idle:
                    return "Idle";
                case ManeuverGizmoInteraction::State::HoverAxis:
                    return "HoverAxis";
                case ManeuverGizmoInteraction::State::DragAxis:
                    return "DragAxis";
            }

            return "Unknown";
        }

        void draw_debug_table_value(const char *label, const char *fmt, ...)
        {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextDisabled("%s", label);
            ImGui::TableSetColumnIndex(1);

            va_list args;
            va_start(args, fmt);
            ImGui::TextV(fmt, args);
            va_end(args);
        }

        void draw_debug_table_bool(const char *label, const bool value)
        {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextDisabled("%s", label);
            ImGui::TableSetColumnIndex(1);
            ImGui::TextColored(value ? ImVec4(0.35f, 0.85f, 0.45f, 1.0f)
                                     : ImVec4(0.72f, 0.72f, 0.72f, 1.0f),
                               "%s",
                               value ? "yes" : "no");
        }

        double timestamp_age_ms(const PredictionDragDebugTelemetry::TimePoint &tp,
                                const PredictionDragDebugTelemetry::TimePoint &now_tp)
        {
            if (!PredictionDragDebugTelemetry::has_time(tp))
            {
                return -1.0;
            }

            return std::chrono::duration<double, std::milli>(now_tp - tp).count();
        }

        void draw_debug_table_age(const char *label,
                                  const PredictionDragDebugTelemetry::TimePoint &tp,
                                  const PredictionDragDebugTelemetry::TimePoint &now_tp)
        {
            const double age_ms = timestamp_age_ms(tp, now_tp);
            if (age_ms < 0.0)
            {
                draw_debug_table_value(label, "n/a");
                return;
            }

            draw_debug_table_value(label, "%.1f ms ago", age_ms);
        }

        void draw_prediction_stage_diag(const char *label,
                                        const OrbitPredictionAdaptiveStageDiagnostics &diag,
                                        const std::size_t sample_count)
        {
            std::string extra{};
            const auto append_extra = [&extra](std::string value) {
                if (!extra.empty())
                {
                    extra += ", ";
                }
                extra += value;
            };
            if (diag.frame_resegmentation_count > 0)
            {
                append_extra("reseg " + std::to_string(diag.frame_resegmentation_count));
            }
            if (diag.maneuver_apply_failed_count > 0)
            {
                append_extra("maneuver failed " +
                             std::to_string(diag.maneuver_apply_failed_count) +
                             " node " +
                             std::to_string(diag.maneuver_apply_failed_node_id));
            }
            const std::string extra_suffix = extra.empty() ? std::string{} : ", " + extra;
            ImGui::Text("%s: req %.2f s, cov %.2f s, seg %zu, samples %zu%s%s",
                        label,
                        diag.requested_duration_s,
                        diag.covered_duration_s,
                        diag.accepted_segments,
                        sample_count,
                        diag.cache_reused ? ", cache" : "",
                        diag.hard_cap_hit ? ", cap" : "");
            ImGui::Text("%s dt min/avg/max: %.6f / %.6f / %.6f, rejected %zu, forced %zu%s",
                        label,
                        diag.min_dt_s,
                        diag.avg_dt_s,
                        diag.max_dt_s,
                        diag.rejected_splits,
                        diag.forced_boundary_splits,
                        extra_suffix.c_str());
        }

        std::string resolve_asset_rel_path(const GameStateContext &ctx, const std::string &rel_path)
        {
            const std::filesystem::path rel(rel_path);
            if (rel.is_absolute())
            {
                return rel.string();
            }
            if (ctx.renderer && ctx.renderer->_assetManager)
            {
                const AssetPaths &paths = ctx.renderer->_assetManager->paths();
                if (!paths.assets.empty())
                {
                    return (paths.assets / rel).string();
                }
            }
            return rel.string();
        }
    } // anonymous namespace

    GameplaySettings GameplayState::extract_settings() const
    {
        GameplaySettings s{};
        s.prediction_draw_full_orbit = _prediction->state().draw_full_orbit;
        s.prediction_draw_future_segment = _prediction->state().draw_future_segment;
        s.prediction_draw_velocity_ray = _prediction->state().draw_velocity_ray;
        s.prediction_line_alpha_scale = _prediction->state().line_alpha_scale;
        s.prediction_line_overlay_boost = _prediction->state().line_overlay_boost;
        s.prediction_periodic_refresh_s = _prediction->state().periodic_refresh_s;
        s.prediction_thrust_refresh_s = _prediction->state().thrust_refresh_s;
        s.prediction_sampling_policy = _prediction->state().sampling_policy;
        s.maneuver_plan_horizon = _maneuver.settings().plan_horizon;
        s.maneuver_plan_windows = _maneuver.settings().plan_windows;
        s.maneuver_plan_live_preview_active = _maneuver.settings().live_preview_active;
        s.orbit_plot_budget = _prediction->budget();
        s.debug_draw_enabled = _debug_draw_enabled;
        s.runtime_orbiter_rails_enabled = _orbit.runtime_orbiter_rails_enabled();
        s.runtime_orbiter_rails_distance_m = _orbit.runtime_orbiter_rails_distance_m();
        s.contact_log_enabled = _contact_log_enabled;
        s.contact_log_print_console = _contact_log_print_console;
        return s;
    }

    void GameplayState::apply_settings(const GameplaySettings &s)
    {
        _prediction->state().draw_full_orbit = s.prediction_draw_full_orbit;
        _prediction->state().draw_future_segment = s.prediction_draw_future_segment;
        _prediction->state().draw_velocity_ray = s.prediction_draw_velocity_ray;
        _prediction->state().line_alpha_scale = s.prediction_line_alpha_scale;
        _prediction->state().line_overlay_boost = s.prediction_line_overlay_boost;
        _prediction->state().periodic_refresh_s = s.prediction_periodic_refresh_s;
        _prediction->state().thrust_refresh_s = s.prediction_thrust_refresh_s;
        _prediction->state().sampling_policy = s.prediction_sampling_policy;
        _maneuver.settings().plan_horizon = s.maneuver_plan_horizon;
        _maneuver.settings().plan_windows = s.maneuver_plan_windows;
        _maneuver.settings().live_preview_active = s.maneuver_plan_live_preview_active;
        _prediction->budget() = s.orbit_plot_budget;
        _debug_draw_enabled = s.debug_draw_enabled;
        _orbit.runtime_orbiter_rails_enabled() = s.runtime_orbiter_rails_enabled;
        _orbit.runtime_orbiter_rails_distance_m() = s.runtime_orbiter_rails_distance_m;
        _contact_log_enabled = s.contact_log_enabled;
        _contact_log_print_console = s.contact_log_print_console;
        mark_prediction_dirty();
    }

    void GameplayState::on_draw_ui(GameStateContext &ctx)
    {
        GameplayPredictionAdapter prediction(build_prediction_access());
        PredictionSubjectStateProvider prediction_subjects =
                PredictionHostContextBuilder(prediction.context()).make_subject_state_provider();

        if (ImGui::BeginMainMenuBar())
        {
            if (ImGui::BeginMenu("View"))
            {
                ImGui::MenuItem("Orbit HUD", nullptr, &_show_orbit_hud);
                ImGui::MenuItem("Orbit Debug Draw", nullptr, &_show_orbit_drag_debug);
                ImGui::MenuItem("Frame View", nullptr, &_show_frame_view);
                ImGui::MenuItem("Maneuver Nodes", nullptr, &_show_maneuver_nodes_panel);
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        if (_show_orbit_hud)
        {
            const ImGuiViewport *viewport = ImGui::GetMainViewport();
            ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + 10, viewport->WorkPos.y + 10));
            ImGui::SetNextWindowBgAlpha(0.4f);

            ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize;

            if (ImGui::Begin("##GameplayHUD", nullptr, flags))
            {
                // ----------------------------------------------------------------
                // Player HUD: sim time, warp, vessel, controls
                // ----------------------------------------------------------------
                const double sim_time_s = _orbit.scenario_owner() ? _orbit.scenario_owner()->sim.time_s() : _fixed_time_s;
                const int sim_hours = static_cast<int>(std::floor(sim_time_s / 3600.0));
                const int sim_minutes = static_cast<int>(std::floor(std::fmod(sim_time_s, 3600.0) / 60.0));
                const double sim_seconds = std::fmod(sim_time_s, 60.0);

                const char *warp_mode = "Realtime";
                switch (_time_warp.mode)
                {
                    case TimeWarpState::Mode::Realtime:
                        warp_mode = "Realtime";
                        break;
                    case TimeWarpState::Mode::PhysicsWarp:
                        warp_mode = "Physics";
                        break;
                    case TimeWarpState::Mode::RailsWarp:
                        warp_mode = "Rails";
                        break;
                }

                ImGui::Text("Sim: %dh %dm %.1fs", sim_hours, sim_minutes, sim_seconds);
                const char *controlled_vessel = "None";
                if (const OrbiterInfo *player_orbiter = _orbit.find_player_orbiter())
                {
                    controlled_vessel = player_orbiter->name.c_str();
                }

                ImGui::Text("Warp: x%.0f (%s)  [,][.] change  [/]/[Backspace] x1", _time_warp.factor(), warp_mode);
                ImGui::Text("Vessel: %s", controlled_vessel);
                ImGui::TextUnformatted("Switch vessel: '[' previous, ']' next");
                ImGui::Text("Real: %.1f s", _elapsed);
                ImGui::Text("[ESC] Pause");

#if !(defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT)
                ImGui::Separator();
                ImGui::TextUnformatted(
                    "WARNING: Built without Jolt physics (collision test requires VULKAN_ENGINE_USE_JOLT=1).");
#endif

                // ----------------------------------------------------------------
                // Scenario save/load
                // ----------------------------------------------------------------
                if (ImGui::Button("Reset scenario"))
                {
                    _reset_requested = true;
                }
                ImGui::SameLine();
                if (ImGui::Button("Replay collision"))
                {
                    _reset_requested = true;
                }

                const std::string scenario_slot_path = resolve_asset_rel_path(ctx, _scenario_slot_rel_path);

                ImGui::SameLine();
                if (ImGui::Button("Save scenario slot"))
                {
                    if (save_scenario_config(scenario_slot_path, _scenario_config))
                    {
                        _scenario_io_status = "Saved scenario: " + scenario_slot_path;
                        _scenario_io_status_ok = true;
                    }
                    else
                    {
                        _scenario_io_status = "Save failed: " + scenario_slot_path;
                        _scenario_io_status_ok = false;
                    }
                }

                ImGui::SameLine();
                if (ImGui::Button("Load scenario slot"))
                {
                    if (auto loaded = load_scenario_config(scenario_slot_path))
                    {
                        _scenario_config = std::move(*loaded);
                        _scenario_io_status = "Loaded scenario: " + scenario_slot_path;
                        _scenario_io_status_ok = true;
                        _reset_requested = true;
                    }
                    else
                    {
                        _scenario_io_status = "Load failed: " + scenario_slot_path;
                        _scenario_io_status_ok = false;
                    }
                }

                ImGui::Text("Scenario slot: %s", scenario_slot_path.c_str());
                if (!_scenario_io_status.empty())
                {
                    if (_scenario_io_status_ok)
                    {
                        ImGui::TextUnformatted(_scenario_io_status.c_str());
                    }
                    else
                    {
                        ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "%s", _scenario_io_status.c_str());
                    }
                }

                // ----------------------------------------------------------------
                // Settings save/load
                // ----------------------------------------------------------------
                const std::string settings_path = resolve_asset_rel_path(ctx, _settings_rel_path);
                if (ImGui::Button("Save settings"))
                {
                    if (save_gameplay_settings(settings_path, extract_settings()))
                    {
                        _settings_io_status = "Saved: " + settings_path;
                        _settings_io_status_ok = true;
                    }
                    else
                    {
                        _settings_io_status = "Save failed: " + settings_path;
                        _settings_io_status_ok = false;
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("Load settings"))
                {
                    if (auto loaded = load_gameplay_settings(settings_path))
                    {
                        apply_settings(*loaded);
                        if (ctx.api)
                        {
                            ctx.api->set_debug_draw_enabled(_debug_draw_enabled);
                        }
                        _settings_io_status = "Loaded: " + settings_path;
                        _settings_io_status_ok = true;
                    }
                    else
                    {
                        _settings_io_status = "Load failed: " + settings_path;
                        _settings_io_status_ok = false;
                    }
                }
                if (!_settings_io_status.empty())
                {
                    if (_settings_io_status_ok)
                    {
                        ImGui::TextUnformatted(_settings_io_status.c_str());
                    }
                    else
                    {
                        ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "%s", _settings_io_status.c_str());
                    }
                }

                // ----------------------------------------------------------------
                // Debug toggles
                // ----------------------------------------------------------------
                ImGui::Checkbox("Contact log", &_contact_log_enabled);
                ImGui::SameLine();
                ImGui::Checkbox("Print console", &_contact_log_print_console);

                if (ctx.api)
                {
                    if (ImGui::Checkbox("Debug draw", &_debug_draw_enabled))
                    {
                        ctx.api->set_debug_draw_enabled(_debug_draw_enabled);
                    }
                }

                bool runtime_rails_enabled = _orbit.runtime_orbiter_rails_enabled();
                if (ImGui::Checkbox("Runtime orbiter rails", &runtime_rails_enabled))
                {
                    _orbit.runtime_orbiter_rails_enabled() = runtime_rails_enabled;
                    mark_prediction_dirty();
                }
                double runtime_rails_distance_m = _orbit.runtime_orbiter_rails_distance_m();
                if (ImGui::DragScalar("Runtime rails distance (m)",
                                      ImGuiDataType_Double,
                                      &runtime_rails_distance_m,
                                      100.0f,
                                      nullptr,
                                      nullptr,
                                      "%.0f"))
                {
                    _orbit.runtime_orbiter_rails_distance_m() = std::max(0.0, runtime_rails_distance_m);
                }

                // ----------------------------------------------------------------
                // Contact log
                // ----------------------------------------------------------------
                ImGui::Separator();
                ImGui::Text("Contacts: %zu", _contact_log.size());

                const int max_lines = 6;
                const int n = static_cast<int>(std::min(_contact_log.size(), static_cast<size_t>(max_lines)));
                for (int i = 0; i < n; ++i)
                {
                    const ContactLogEntry &e = _contact_log[_contact_log.size() - 1 - static_cast<size_t>(i)];
                    ImGui::Text("[%s][%.2fs] self=%u other=%u depth=%.3f p=(%.2f,%.2f,%.2f)",
                                contact_event_type_name(e.type),
                                e.time_s,
                                e.self_body,
                                e.other_body,
                                e.penetration_depth,
                                e.point.x, e.point.y, e.point.z);
                }

                // ----------------------------------------------------------------
                // Ship controller HUD
                // ----------------------------------------------------------------
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
                {
                    const EntityId player_eid = _orbit.player_entity();
                    if (player_eid.is_valid())
                    {
                        Entity *player = _world.entities().find(player_eid);
                        if (player)
                        {
                            auto *sc = player->get_component<ShipController>();
                            if (sc)
                            {
                                ImGui::Separator();
                                const bool rails_warp =
                                        _orbital_physics.rails_warp_active() &&
                                        _time_warp.mode == TimeWarpState::Mode::RailsWarp;
                                const glm::vec3 td = rails_warp
                                                             ? _orbital_physics.rails_last_thrust_dir_local()
                                                             : sc->last_thrust_dir();
                                ImGui::Text("SAS: %s  [T] toggle", sc->sas_enabled() ? "ON " : "OFF");
                                ImGui::Text("Thrust input: (%.1f, %.1f, %.1f)%s",
                                            td.x, td.y, td.z,
                                            (rails_warp && _orbital_physics.rails_thrust_applied_this_tick()) ? " [applied]" : "");

                                if (rails_warp)
                                {
                                    WorldVec3 ship_pos_world{0.0, 0.0, 0.0};
                                    glm::dvec3 ship_vel_world(0.0);
                                    glm::vec3 ship_vel_local_f(0.0f);
                                    if (prediction_subjects.get_player_world_state(ship_pos_world,
                                                                                   ship_vel_world,
                                                                                   ship_vel_local_f))
                                    {
                                        ImGui::Text("Speed(world): %.2f m/s", glm::length(ship_vel_world));
                                    }
                                }
                                else if (player->has_physics() && _physics)
                                {
                                    const Physics::BodyId body_id{player->physics_body_value()};
                                    if (_physics->is_body_valid(body_id))
                                    {
                                        const Physics::MotionType motion = _physics->get_motion_type(body_id);
                                        const char *motion_str =
                                                (motion == Physics::MotionType::Dynamic)
                                                    ? "Dynamic"
                                                    : (motion == Physics::MotionType::Kinematic) ? "Kinematic (forces ignored)" : "Static";
                                        ImGui::Text("Motion: %s", motion_str);

                                        float thrust = sc->thrust_force();
                                        if (ImGui::DragFloat("Thrust force (N)", &thrust, 1000.0f, 0.0f, 1.0e9f, "%.1f"))
                                        {
                                            sc->set_thrust_force(thrust);
                                        }

                                        float torque = sc->torque_strength();
                                        if (ImGui::DragFloat("Torque strength (N*m)", &torque, 1000.0f, 0.0f, 1.0e9f, "%.1f"))
                                        {
                                            sc->set_torque_strength(torque);
                                        }

                                        float sas = sc->sas_damping();
                                        if (ImGui::DragFloat("SAS damping", &sas, 0.1f, 0.0f, 1.0e4f, "%.2f"))
                                        {
                                            sc->set_sas_damping(sas);
                                        }

                                        const glm::vec3 vel = _physics->get_linear_velocity(body_id);
                                        ImGui::Text("Speed(local): %.2f m/s", glm::length(vel));
                                        if (_physics_context)
                                        {
                                            const glm::dvec3 v_world = _physics_context->velocity_origin_world() + glm::dvec3(vel);
                                            ImGui::Text("Speed(world): %.2f m/s", glm::length(v_world));
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
#endif

                {
                    const EntityId player_eid = _orbit.player_entity();
                    if (player_eid.is_valid())
                    {
                        const Entity *player = _world.entities().find(player_eid);
                        if (player)
                        {
                            ImGui::Separator();
                            const glm::vec3 com_local = player->physics_origin_offset_local();
                            const float alpha_f = std::clamp(ctx.interpolation_alpha(), 0.0f, 1.0f);
                            const WorldVec3 model_origin_world = player->get_render_position_world(alpha_f);
                            const WorldVec3 com_world = player->get_render_physics_center_of_mass_world(alpha_f);
                            ImGui::Text("Model origin(world): %.2f, %.2f, %.2f",
                                        model_origin_world.x, model_origin_world.y, model_origin_world.z);
                            ImGui::Text("COM local: %.3f, %.3f, %.3f",
                                        com_local.x, com_local.y, com_local.z);
                            ImGui::Text("COM world: %.2f, %.2f, %.2f",
                                        com_world.x, com_world.y, com_world.z);
                        }
                    }
                }

                // ================================================================
                // Orbit section
                // ================================================================
                ImGui::Separator();
                if (ImGui::CollapsingHeader("Orbit", ImGuiTreeNodeFlags_DefaultOpen))
                {
                    OrbitPlotSystem *orbit_plot =
                            (ctx.renderer && ctx.renderer->_context) ? ctx.renderer->_context->orbit_plot : nullptr;
                    prediction.rebuild_prediction_subjects();
                    prediction.rebuild_prediction_frame_options();
                    prediction.rebuild_prediction_analysis_options();

                    // --- Key orbital info (always visible) ---
                    const PredictionTrackState *active_prediction = prediction.active_prediction_track();
                    std::string active_prediction_label = active_prediction
                                                                ? prediction.prediction_subject_label(active_prediction->key)
                                                                : std::string("None");
                    ImGui::Text("Focused subject: %s", active_prediction_label.c_str());

                    // Display frame / Analysis frame combos
                    const char *frame_label = (_prediction->state().frame_selection.selected_index >= 0 &&
                                               _prediction->state().frame_selection.selected_index <
                                                       static_cast<int>(_prediction->state().frame_selection.options.size()))
                                                  ? _prediction->state().frame_selection.options[static_cast<size_t>(
                                                            _prediction->state().frame_selection.selected_index)].label.c_str()
                                                  : "Unknown";
                    if (ImGui::BeginCombo("Display frame", frame_label))
                    {
                        for (std::size_t i = 0; i < _prediction->state().frame_selection.options.size(); ++i)
                        {
                            const PredictionFrameOption &option = _prediction->state().frame_selection.options[i];
                            const bool selected =
                                    option.spec.type == _prediction->state().frame_selection.spec.type &&
                                    option.spec.primary_body_id == _prediction->state().frame_selection.spec.primary_body_id &&
                                    option.spec.secondary_body_id == _prediction->state().frame_selection.spec.secondary_body_id &&
                                    option.spec.target_spacecraft_id == _prediction->state().frame_selection.spec.target_spacecraft_id;
                            if (ImGui::Selectable(option.label.c_str(), selected))
                            {
                                (void) prediction.set_prediction_frame_spec(option.spec);
                            }
                            if (selected)
                            {
                                ImGui::SetItemDefaultFocus();
                            }
                        }
                        ImGui::EndCombo();
                    }

                    const char *analysis_label =
                            (_prediction->state().analysis_selection.selected_index >= 0 &&
                             _prediction->state().analysis_selection.selected_index <
                                     static_cast<int>(_prediction->state().analysis_selection.options.size()))
                                ? _prediction->state().analysis_selection
                                          .options[static_cast<size_t>(_prediction->state().analysis_selection.selected_index)]
                                          .label.c_str()
                                : "Unknown";
                    if (ImGui::BeginCombo("Analysis frame", analysis_label))
                    {
                        for (std::size_t i = 0; i < _prediction->state().analysis_selection.options.size(); ++i)
                        {
                            const PredictionAnalysisOption &option = _prediction->state().analysis_selection.options[i];
                            const bool selected =
                                    option.spec.mode == _prediction->state().analysis_selection.spec.mode &&
                                    option.spec.fixed_body_id == _prediction->state().analysis_selection.spec.fixed_body_id;
                            if (ImGui::Selectable(option.label.c_str(), selected))
                            {
                                (void) prediction.set_prediction_analysis_spec(option.spec);
                            }
                            if (selected)
                            {
                                ImGui::SetItemDefaultFocus();
                            }
                        }
                        ImGui::EndCombo();
                    }

                    // Orbital metrics
                    WorldVec3 subject_pos_world{0.0, 0.0, 0.0};
                    glm::dvec3 subject_vel_world(0.0);
                    glm::vec3 subject_vel_local_f(0.0f);

                    active_prediction = prediction.active_prediction_track();
                    active_prediction_label = active_prediction
                                                  ? prediction.prediction_subject_label(active_prediction->key)
                                                  : std::string("None");
                    const bool have_subject =
                            active_prediction &&
                            prediction_subjects.get_subject_world_state(active_prediction->key,
                                                                        subject_pos_world,
                                                                        subject_vel_world,
                                                                        subject_vel_local_f);
                    if (!have_subject)
                    {
                        ImGui::TextUnformatted("Prediction subject state unavailable.");
                    }
                    else
                    {
                        if (active_prediction && active_prediction->cache.identity.valid)
                        {
                            if (!active_prediction->cache.analysis.altitude_km.empty())
                            {
                                ImGui::Text("Altitude: %.0f m",
                                            static_cast<double>(active_prediction->cache.analysis.altitude_km.front()) *
                                                    1000.0);
                            }
                            if (!active_prediction->cache.analysis.speed_kmps.empty())
                            {
                                ImGui::Text("Speed:    %.3f km/s",
                                            static_cast<double>(active_prediction->cache.analysis.speed_kmps.front()));
                            }

                            if (!active_prediction->is_celestial)
                            {
                                ImGui::Text("Predicted Pe: %.1f km",
                                            active_prediction->cache.analysis.periapsis_alt_km);
                                if (std::isfinite(active_prediction->cache.analysis.apoapsis_alt_km))
                                {
                                    ImGui::Text("Predicted Ap: %.1f km",
                                                active_prediction->cache.analysis.apoapsis_alt_km);
                                }
                                else
                                {
                                    ImGui::TextUnformatted("Predicted Ap: escape");
                                }

                                if (active_prediction->cache.analysis.orbital_period_s > 0.0 &&
                                    std::isfinite(active_prediction->cache.analysis.orbital_period_s))
                                {
                                    ImGui::Text("Predicted Period: %.2f min",
                                                active_prediction->cache.analysis.orbital_period_s / 60.0);
                                }
                            }
                        }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
                        const EntityId player_eid = _orbit.player_entity();
                        if (_physics && _physics_context && player_eid.is_valid() &&
                            active_prediction &&
                            prediction.prediction_subject_is_player(active_prediction->key))
                        {
                            const glm::dvec3 v_origin_world = _physics_context->velocity_origin_world();
                            ImGui::Text("v_origin: %.1f, %.1f, %.1f m/s", v_origin_world.x, v_origin_world.y,
                                        v_origin_world.z);
                            ImGui::Text("v_local:  %.2f, %.2f, %.2f m/s", subject_vel_local_f.x, subject_vel_local_f.y,
                                        subject_vel_local_f.z);

                            const Entity *player = _world.entities().find(player_eid);
                            if (player && player->has_physics())
                            {
                                const Physics::BodyId body_id{player->physics_body_value()};
                                if (_physics->is_body_valid(body_id))
                                {
                                    const glm::vec3 w_local_f = _physics->get_angular_velocity(body_id);
                                    ImGui::Text("w_local:  %.3f, %.3f, %.3f rad/s (|w|=%.3f)",
                                                w_local_f.x, w_local_f.y, w_local_f.z, glm::length(w_local_f));
                                }
                            }
                        }
#endif
                    }

                    // --- Orbit View (collapsed by default) ---
                    if (ImGui::CollapsingHeader("Orbit View"))
                    {
                    ImGui::Checkbox("Prediction full orbit", &_prediction->state().draw_full_orbit);
                    ImGui::Checkbox("Prediction future segment", &_prediction->state().draw_future_segment);
                    ImGui::Checkbox("Prediction velocity ray", &_prediction->state().draw_velocity_ray);

                    float prediction_alpha_scale = _prediction->state().line_alpha_scale;
                    if (ImGui::DragFloat("Prediction line alpha scale",
                                         &prediction_alpha_scale,
                                         0.05f,
                                         0.1f,
                                         8.0f,
                                         "%.2f"))
                    {
                        _prediction->state().line_alpha_scale = std::clamp(prediction_alpha_scale, 0.1f, 8.0f);
                    }

                    float prediction_overlay_boost = _prediction->state().line_overlay_boost;
                    if (ImGui::DragFloat("Prediction line overlay boost",
                                         &prediction_overlay_boost,
                                         0.01f,
                                         0.0f,
                                         1.0f,
                                         "%.2f"))
                    {
                        _prediction->state().line_overlay_boost = std::clamp(prediction_overlay_boost, 0.0f, 1.0f);
                    }
                    ImGui::SameLine();
                    ImGui::TextUnformatted("(0 = depth-only)");
                }

                    // --- Orbit Debug (collapsed by default) ---
                    if (ImGui::CollapsingHeader("Orbit Debug"))
                    {
                        ImGui::TextUnformatted("Plan horizon lives in Maneuver Nodes.");

                        float refresh_s = static_cast<float>(_prediction->state().periodic_refresh_s);
                        if (ImGui::DragFloat("Prediction refresh (s)", &refresh_s, 1.0f, 0.0f, 36000.0f, "%.1f"))
                        {
                            _prediction->state().periodic_refresh_s = static_cast<double>(std::max(0.0f, refresh_s));
                        }
                        ImGui::SameLine();
                        ImGui::TextUnformatted("(0 = never)");

                        float thrust_refresh_s = static_cast<float>(_prediction->state().thrust_refresh_s);
                        if (ImGui::DragFloat("Prediction thrust refresh (s)",
                                             &thrust_refresh_s,
                                             0.01f,
                                             0.0f,
                                             2.0f,
                                             "%.2f"))
                        {
                            _prediction->state().thrust_refresh_s = static_cast<double>(std::max(0.0f, thrust_refresh_s));
                        }
                    ImGui::SameLine();
                    ImGui::TextUnformatted("(0 = every fixed tick)");

                    ImGui::SeparatorText("Prediction Policy");

                    float orbiter_min_window_s = static_cast<float>(_prediction->state().sampling_policy.orbiter_min_window_s);
                    if (ImGui::DragFloat("Orbiter min window (s)",
                                         &orbiter_min_window_s,
                                         10.0f,
                                         0.0f,
                                         15552000.0f,
                                         "%.0f"))
                    {
                        _prediction->state().sampling_policy.orbiter_min_window_s =
                                static_cast<double>(std::max(0.0f, orbiter_min_window_s));
                    }

                    float celestial_min_window_s = static_cast<float>(_prediction->state().sampling_policy.celestial_min_window_s);
                    if (ImGui::DragFloat("Celestial min window (s)",
                                         &celestial_min_window_s,
                                         60.0f,
                                         0.0f,
                                         15552000.0f,
                                         "%.0f"))
                    {
                        _prediction->state().sampling_policy.celestial_min_window_s =
                                static_cast<double>(std::max(0.0f, celestial_min_window_s));
                    }

                    ImGui::Text("Plan horizon: %.0f s", _maneuver.settings().plan_horizon.horizon_s);

                    ImGui::SeparatorText("Orbit Budget");

                    float render_error_px = static_cast<float>(_prediction->budget().render_error_px);
                    if (ImGui::DragFloat("Render error (px)", &render_error_px, 0.01f, 0.05f, 4.0f, "%.2f"))
                    {
                        _prediction->budget().render_error_px = std::clamp(static_cast<double>(render_error_px), 0.05, 4.0);
                    }

                    int render_max_segments_cpu = _prediction->budget().render_max_segments_cpu;
                    if (ImGui::DragInt("Render max segments (CPU)",
                                       &render_max_segments_cpu,
                                       50.0f,
                                       64,
                                       200000))
                    {
                        _prediction->budget().render_max_segments_cpu = std::clamp(render_max_segments_cpu, 64, 200000);
                    }

                    int pick_max_segments = _prediction->budget().pick_max_segments;
                    if (ImGui::DragInt("Pick max segments", &pick_max_segments, 50.0f, 64, 32000))
                    {
                        _prediction->budget().pick_max_segments = std::clamp(pick_max_segments, 64, 32000);
                    }

                    float pick_margin_ratio = static_cast<float>(_prediction->budget().pick_frustum_margin_ratio);
                    if (ImGui::DragFloat("Pick frustum margin ratio",
                                         &pick_margin_ratio,
                                         0.01f,
                                         0.0f,
                                         1.0f,
                                         "%.2f"))
                    {
                        _prediction->budget().pick_frustum_margin_ratio =
                                std::clamp(static_cast<double>(pick_margin_ratio), 0.0, 1.0);
                    }

                    if (orbit_plot)
                    {
                        int upload_budget_mib = static_cast<int>(std::clamp<std::size_t>(
                                orbit_plot->settings().upload_budget_bytes / (1024ull * 1024ull),
                                1ull,
                                256ull));
                        if (ImGui::SliderInt("Orbit upload budget (MiB)", &upload_budget_mib, 1, 256))
                        {
                            orbit_plot->settings().upload_budget_bytes =
                                    static_cast<std::size_t>(upload_budget_mib) * 1024ull * 1024ull;
                        }
                    }
                }

                    // --- Orbit Performance (collapsed by default) ---
                    if (orbit_plot && ImGui::CollapsingHeader("Orbit Performance"))
                    {
                    const OrbitPlotSystem::Stats &plot_stats = orbit_plot->stats();
                    const OrbitPlotPerfStats &perf = _prediction->state().orbit_plot_perf;
                    const PredictionTrackState *active_track = prediction.active_prediction_track();
                    const double upload_mib =
                            static_cast<double>(plot_stats.upload_bytes_last_frame) / (1024.0 * 1024.0);
                    const double budget_mib =
                            static_cast<double>(plot_stats.upload_budget_bytes) / (1024.0 * 1024.0);
                    const double peak_mib =
                            static_cast<double>(plot_stats.upload_bytes_peak) / (1024.0 * 1024.0);

                    ImGui::Text("Visible subjects: %zu", _prediction->state().tracks.size());
                    ImGui::Text("Solver segments (base/planned): %u / %u",
                                perf.solver_segments_base,
                                perf.solver_segments_planned);
                    if (active_track)
                    {
                        const OrbitPredictionDiagnostics &solver_diag = active_track->solver_diagnostics;
                        const OrbitPredictionDerivedDiagnostics &derived_diag = active_track->derived_diagnostics;
                        ImGui::Text("Prediction status (solver/frame): %s / %s",
                                    prediction_solver_status_label(solver_diag.status),
                                    prediction_derived_status_label(derived_diag.status));
                        draw_prediction_stage_diag("Ephemeris", solver_diag.ephemeris, 0);
                        draw_prediction_stage_diag(
                                "Solver base",
                                solver_diag.trajectory_base,
                                solver_diag.trajectory_sample_count);
                        if (solver_diag.trajectory_planned.accepted_segments > 0 ||
                            solver_diag.trajectory_sample_count_planned > 0 ||
                            solver_diag.trajectory_planned.maneuver_apply_failed_count > 0)
                        {
                            draw_prediction_stage_diag(
                                    "Solver planned",
                                    solver_diag.trajectory_planned,
                                    solver_diag.trajectory_sample_count_planned);
                        }
                        draw_prediction_stage_diag(
                                "Frame base",
                                derived_diag.frame_base,
                                derived_diag.frame_sample_count);
                        if (derived_diag.frame_planned.accepted_segments > 0 ||
                            derived_diag.frame_sample_count_planned > 0)
                        {
                            draw_prediction_stage_diag(
                                    "Frame planned",
                                    derived_diag.frame_planned,
                                    derived_diag.frame_sample_count_planned);
                        }
                    }
                    ImGui::Text("Orbit lines (active/pending): %u / %u",
                                plot_stats.active_line_count,
                                plot_stats.pending_line_count);
                    ImGui::Text("Orbit segments (depth/overlay): %u / %u",
                                plot_stats.depth_segment_count,
                                plot_stats.overlay_segment_count);
                    ImGui::Text("Pick segments (before/after): %u / %u",
                                perf.pick_segments_before_cull,
                                perf.pick_segments);

                    ImGui::Text("Timing ms (solver/render_lod/pick_lod/upload): %.3f / %.3f / %.3f / %.3f",
                                perf.solver_ms_last,
                                perf.render_lod_ms_last,
                                perf.pick_lod_ms_last,
                                plot_stats.upload_ms_last_frame);
                    ImGui::Text("Orbit upload: %.2f MiB / %.2f MiB%s",
                                upload_mib,
                                budget_mib,
                                plot_stats.upload_cap_hit_last_frame ? " [cap]" : "");
                    ImGui::Text("Cap hits (render/pick/upload): %llu / %llu / %llu",
                                static_cast<unsigned long long>(perf.render_cap_hits_total),
                                static_cast<unsigned long long>(perf.pick_cap_hits_total),
                                static_cast<unsigned long long>(plot_stats.upload_cap_hits_total));
                    ImGui::Text("Orbit upload peak: %.2f MiB, upload ms peak: %.3f",
                                peak_mib,
                                plot_stats.upload_ms_peak);

                    if (_prediction->state().orbit_plot_perf.planned_window_valid || !_maneuver.plan().nodes.empty())
                    {
                        ImGui::Separator();
                        ImGui::Text("Planned window: %s", _prediction->state().orbit_plot_perf.planned_window_valid ? "VALID" : "INVALID");
                        ImGui::Text("  t0p (traj start): %.3f", _prediction->state().orbit_plot_perf.planned_window_t0p);
                        ImGui::Text("  now_s:            %.3f", _prediction->state().orbit_plot_perf.planned_window_now_s);
                        ImGui::Text("  anchor (node):    %.3f", _prediction->state().orbit_plot_perf.planned_window_anchor_s);
                        ImGui::Text("  t_plan_start:     %.3f", _prediction->state().orbit_plot_perf.planned_window_t_start);
                        ImGui::Text("  t_plan_end:       %.3f", _prediction->state().orbit_plot_perf.planned_window_t_end);
                        if (_prediction->state().orbit_plot_perf.planned_window_valid)
                        {
                            ImGui::Text("  offset from t0p:  %.3f s", _prediction->state().orbit_plot_perf.planned_window_t_start - _prediction->state().orbit_plot_perf.planned_window_t0p);
                            ImGui::Text("  offset from now:  %.3f s", _prediction->state().orbit_plot_perf.planned_window_t_start - _prediction->state().orbit_plot_perf.planned_window_now_s);
                        }
                    }
                }

                    // --- Physics Debug (collapsed by default) ---
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
                    if (_physics && _physics_context && ImGui::CollapsingHeader("Physics Debug"))
                    {
                    using VelocityOriginMode = OrbitalPhysicsSystem::VelocityOriginMode;
                    int mode_idx = (_orbital_physics.velocity_origin_mode() == VelocityOriginMode::PerStepAnchorSync) ? 0 : 1;
                    const char *modes[] = {"Per-step anchor sync", "Free-fall anchor frame"};
                    if (ImGui::Combo("Velocity origin mode", &mode_idx, modes, IM_ARRAYSIZE(modes)))
                    {
                        _orbital_physics.set_velocity_origin_mode(
                                (mode_idx == 0)
                                    ? VelocityOriginMode::PerStepAnchorSync
                                    : VelocityOriginMode::FreeFallAnchorFrame);
                        mark_prediction_dirty();
                    }

                    GameWorld::RebaseSettings rs = _world.rebase_settings();
                    float v_rebase = static_cast<float>(rs.velocity_threshold_mps);
                    if (ImGui::DragFloat("Velocity rebase threshold (m/s)", &v_rebase, 50.0f, 0.0f, 100000.0f, "%.1f"))
                    {
                        rs.velocity_threshold_mps = static_cast<double>(std::max(0.0f, v_rebase));
                        _world.set_rebase_settings(rs);
                    }
                    ImGui::SameLine();
                    ImGui::TextUnformatted("(0 = off)");

                    const EntityId player_eid = _orbit.player_entity();
                    if (_physics && player_eid.is_valid())
                    {
                        const Entity *player = _world.entities().find(player_eid);
                        if (player && player->has_physics())
                        {
                            const Physics::BodyId body_id{player->physics_body_value()};
                            if (_physics->is_body_valid(body_id))
                            {
                                const Physics::MotionType motion = _physics->get_motion_type(body_id);
                                bool kinematic = motion == Physics::MotionType::Kinematic;
                                if (ImGui::Checkbox("Controlled vessel kinematic", &kinematic))
                                {
                                    const Physics::MotionType target =
                                            kinematic ? Physics::MotionType::Kinematic : Physics::MotionType::Dynamic;
                                    (void) _physics->set_motion_type(body_id, target);
                                }
                                ImGui::SameLine();
                                ImGui::TextUnformatted("Controlled vessel is also the rebase anchor.");
                            }
                        }
                    }
                    }
#endif
                } // end Orbit CollapsingHeader
            }
            ImGui::End();
        }

        ManeuverUiController::Context maneuver_ui = build_maneuver_ui_context(ctx);
        ManeuverUiController::open_nodes_panel_from_orbit_pick_release(maneuver_ui);

        if (_show_maneuver_nodes_panel)
        {
            ManeuverUiController::draw_nodes_panel(maneuver_ui);
        }
        ManeuverUiController::draw_imgui_gizmo(maneuver_ui);
        if (_show_orbit_drag_debug)
        {
            draw_orbit_drag_debug_window(ctx);
        }
        if (_show_frame_view)
        {
            _frame_monitor.draw_ui();
        }
    }

    void GameplayState::draw_orbit_drag_debug_window(GameStateContext &ctx)
    {
        GameplayPredictionAdapter prediction(build_prediction_access());
        OrbitPlotSystem *orbit_plot =
                (ctx.renderer && ctx.renderer->_context) ? ctx.renderer->_context->orbit_plot : nullptr;
        const OrbitPlotSystem::Stats *plot_stats = orbit_plot ? &orbit_plot->stats() : nullptr;

        const ImGuiViewport *viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(
                ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - 560.0f, viewport->WorkPos.y + 16.0f),
                ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(540.0f, 620.0f), ImGuiCond_FirstUseEver);

        bool window_open = _show_orbit_drag_debug;
        if (!ImGui::Begin("Orbit Debug Draw", &window_open))
        {
            _show_orbit_drag_debug = window_open;
            ImGui::End();
            return;
        }
        _show_orbit_drag_debug = window_open;

        ImGui::SeparatorText("Draw Controls");
        bool game_debug_layer_enabled = false;
        if (ctx.api)
        {
            bool debug_draw_enabled = ctx.api->get_debug_draw_enabled();
            _debug_draw_enabled = debug_draw_enabled;
            if (ImGui::Checkbox("Global debug draw", &debug_draw_enabled))
            {
                _debug_draw_enabled = debug_draw_enabled;
                ctx.api->set_debug_draw_enabled(_debug_draw_enabled);
            }

            bool show_depth_tested = ctx.api->get_debug_show_depth_tested();
            ImGui::SameLine();
            if (ImGui::Checkbox("Depth", &show_depth_tested))
            {
                ctx.api->set_debug_show_depth_tested(show_depth_tested);
            }

            bool show_overlay = ctx.api->get_debug_show_overlay();
            ImGui::SameLine();
            if (ImGui::Checkbox("Overlay", &show_overlay))
            {
                ctx.api->set_debug_show_overlay(show_overlay);
            }

            uint32_t debug_layer_mask = ctx.api->get_debug_layer_mask();
            const uint32_t game_debug_layer = static_cast<uint32_t>(DebugDrawLayer::Misc);
            game_debug_layer_enabled = (debug_layer_mask & game_debug_layer) != 0u;
            ImGui::SameLine();
            if (ImGui::Checkbox("Game layer", &game_debug_layer_enabled))
            {
                if (game_debug_layer_enabled)
                {
                    debug_layer_mask |= game_debug_layer;
                }
                else
                {
                    debug_layer_mask &= ~game_debug_layer;
                }
                ctx.api->set_debug_layer_mask(debug_layer_mask);
            }
        }
        else
        {
            ImGui::TextDisabled("Game API unavailable; world-space debug draw controls are disabled.");
        }

        if (orbit_plot)
        {
            ImGui::Checkbox("Orbit plot pass", &orbit_plot->settings().enabled);

            float line_width_px = orbit_plot->settings().line_width_px;
            if (ImGui::DragFloat("Orbit line width (px)", &line_width_px, 0.1f, 1.0f, 8.0f, "%.1f"))
            {
                orbit_plot->settings().line_width_px = std::clamp(line_width_px, 1.0f, 8.0f);
            }

            float line_aa_px = orbit_plot->settings().line_aa_px;
            if (ImGui::DragFloat("Orbit line AA (px)", &line_aa_px, 0.1f, 0.0f, 4.0f, "%.1f"))
            {
                orbit_plot->settings().line_aa_px = std::clamp(line_aa_px, 0.0f, 4.0f);
            }
        }
        else
        {
            ImGui::TextDisabled("OrbitPlotSystem unavailable.");
        }

        ImGui::Checkbox("Velocity ray", &_prediction->state().draw_velocity_ray);
        ImGui::SameLine();
        ImGui::Checkbox("Planned dashed", &_prediction->state().draw_config.draw_planned_as_dashed);
        ImGui::SameLine();
        ImGui::BeginDisabled(!_maneuver.settings().nodes_enabled);
        ImGui::Checkbox("Node overlay", &_maneuver.settings().nodes_debug_draw);
        ImGui::EndDisabled();

        const bool node_overlay_effective =
                ctx.api &&
                _debug_draw_enabled &&
                game_debug_layer_enabled &&
                _maneuver.settings().nodes_enabled &&
                _maneuver.settings().nodes_debug_draw;
        const bool velocity_ray_effective =
                ctx.api &&
                _debug_draw_enabled &&
                game_debug_layer_enabled &&
                _prediction->state().draw_velocity_ray;
        if (ImGui::BeginTable("##orbit_debug_draw_effective", 2,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                      ImGuiTableFlags_SizingStretchProp))
        {
            draw_debug_table_bool("Game debug layer", game_debug_layer_enabled);
            draw_debug_table_bool("Velocity ray effective", velocity_ray_effective);
            draw_debug_table_bool("Node overlay effective", node_overlay_effective);
            draw_debug_table_bool("Maneuver nodes enabled", _maneuver.settings().nodes_enabled);
            draw_debug_table_bool("Orbit plot active", orbit_plot && orbit_plot->settings().enabled);
            ImGui::EndTable();
        }

        const PredictionTrackState *active_track = prediction.active_prediction_track();
        if (!active_track)
        {
            ImGui::SeparatorText("Runtime");
            ImGui::TextUnformatted("No active prediction track.");
            ImGui::End();
            return;
        }

        const PredictionDragDebugTelemetry &debug = active_track->drag_debug;
        const auto now_tp = PredictionDragDebugTelemetry::Clock::now();
        const std::string subject_label = prediction.prediction_subject_label(active_track->key);
        const bool active_track_with_maneuvers =
                active_track->supports_maneuvers &&
                _maneuver.settings().nodes_enabled &&
                !_maneuver.plan().nodes.empty();
        const bool live_preview_effective = _maneuver.live_preview_active(active_track_with_maneuvers);

        const orbitsim::TrajectoryFrameSpec frame_spec =
                active_track->cache.display.resolved_frame_spec_valid
                        ? active_track->cache.display.resolved_frame_spec
                        : _prediction->state().frame_selection.spec;
        const bool live_chunk_path_supported =
                frame_spec.type != orbitsim::TrajectoryFrameType::Inertial &&
                frame_spec.type != orbitsim::TrajectoryFrameType::LVLH;
        const bool have_sim_now = _orbit.scenario_owner() != nullptr;
        const double sim_now_s = have_sim_now ? _orbit.scenario_owner()->sim.time_s() : 0.0;
        const bool have_build_time = active_track->cache.identity.valid && have_sim_now;
        const double sim_since_build_s =
                have_build_time ? std::max(0.0, sim_now_s - active_track->cache.identity.build_time_s) : 0.0;
        const double drag_gate_remaining_ms =
                PredictionDragDebugTelemetry::has_time(debug.last_request_tp)
                        ? std::max(0.0,
                                   OrbitPredictionTuning::kDragRebuildMinIntervalS -
                                           std::chrono::duration<double>(now_tp - debug.last_request_tp).count()) *
                                  1000.0
                        : 0.0;

        double total_plan_dv_mps = 0.0;
        std::size_t valid_gizmo_count = 0;
        for (const ManeuverNode &node : _maneuver.plan().nodes)
        {
            total_plan_dv_mps += node.total_dv_mps;
            if (node.gizmo_valid)
            {
                ++valid_gizmo_count;
            }
        }

        ImGui::SeparatorText("Prediction Runtime");
        if (ImGui::BeginTable("##orbit_debug_prediction_runtime", 2,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                      ImGuiTableFlags_SizingStretchProp))
        {
            draw_debug_table_value("Subject", "%s", subject_label.c_str());
            draw_debug_table_value("Solver / frame", "%s / %s",
                                   prediction_solver_status_label(active_track->solver_diagnostics.status),
                                   prediction_derived_status_label(active_track->derived_diagnostics.status));
            draw_debug_table_value("Preview state", "%s", prediction_preview_state_label(active_track->preview_state));
            draw_debug_table_value("Result quality", "%s",
                                   prediction_solve_quality_label(debug.last_result_solve_quality));
            draw_debug_table_value("Last request quality", "%s",
                                   prediction_solve_quality_label(debug.last_request_solve_quality));
            draw_debug_table_value("Last request window / impulses", "%.3f s / %zu",
                                   debug.last_request_future_window_s,
                                   debug.last_request_maneuver_count);
            draw_debug_table_bool("Last request preview patch", debug.last_request_preview_patch);
            draw_debug_table_bool("Last request full stream", debug.last_request_full_stream_publish);
            draw_debug_table_value("Pending solver / derived / dirty", "%s / %s / %s",
                                   active_track->request_pending ? "yes" : "no",
                                   active_track->derived_request_pending ? "yes" : "no",
                                   active_track->dirty ? "yes" : "no");
            draw_debug_table_bool("Invalidated while pending", active_track->invalidated_while_pending);
            draw_debug_table_value("Cache generation", "%llu visible / %llu authoritative",
                                   static_cast<unsigned long long>(active_track->cache.identity.generation_id),
                                   static_cast<unsigned long long>(
                                           active_track->authoritative_cache.identity.generation_id));
            draw_debug_table_bool("Visible cache valid", active_track->cache.identity.valid);
            draw_debug_table_bool("Authoritative cache valid", active_track->authoritative_cache.identity.valid);
            if (have_build_time)
            {
                draw_debug_table_value("Sim since cache build", "%.3f s", sim_since_build_s);
            }
            draw_debug_table_value("Drag rebuild gate", "%.1f ms", drag_gate_remaining_ms);
            draw_debug_table_bool("Live chunk path", live_chunk_path_supported);
            draw_debug_table_bool("Supports maneuvers", active_track->supports_maneuvers);
            ImGui::EndTable();
        }

        ImGui::SeparatorText("Gizmo & Plan");
        if (ImGui::BeginTable("##orbit_debug_gizmo_plan", 2,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                      ImGuiTableFlags_SizingStretchProp))
        {
            draw_debug_table_value("Gizmo state", "%s",
                                   gizmo_interaction_state_label(_maneuver.gizmo_interaction().state));
            draw_debug_table_value("Nodes / valid gizmos", "%zu / %zu",
                                   _maneuver.plan().nodes.size(),
                                   valid_gizmo_count);
            draw_debug_table_value("Selected node", "%d", _maneuver.plan().selected_node_id);
            draw_debug_table_value("Plan revision", "%llu",
                                   static_cast<unsigned long long>(_maneuver.revision()));
            draw_debug_table_value("Total DV", "%.3f m/s", total_plan_dv_mps);
            draw_debug_table_bool("Live preview setting", _maneuver.settings().live_preview_active);
            draw_debug_table_bool("Live preview effective", live_preview_effective);
            draw_debug_table_value("Plan horizon", "%.0f s", _maneuver.settings().plan_horizon.horizon_s);
            draw_debug_table_value("Preview window", "%.0f s", _maneuver.settings().plan_windows.preview_window_s);
            draw_debug_table_bool("Warp to node active", _maneuver.runtime().warp_to_time_active);
            draw_debug_table_value("Armed execute node", "%d", _maneuver.runtime().execute_node_id);
            ImGui::EndTable();
        }

        if (_maneuver.gizmo_interaction().node_id >= 0)
        {
            ImGui::Text("Gizmo node/axis: %d / %s",
                        _maneuver.gizmo_interaction().node_id,
                        Gizmo::maneuver_axis_label(_maneuver.settings().gizmo_basis_mode, _maneuver.gizmo_interaction().axis));
        }

        if (plot_stats)
        {
            const double upload_mib =
                    static_cast<double>(plot_stats->upload_bytes_last_frame) / (1024.0 * 1024.0);
            const double budget_mib =
                    static_cast<double>(plot_stats->upload_budget_bytes) / (1024.0 * 1024.0);
            const double peak_mib =
                    static_cast<double>(plot_stats->upload_bytes_peak) / (1024.0 * 1024.0);
            const OrbitPlotPerfStats &perf = _prediction->state().orbit_plot_perf;

            ImGui::SeparatorText("Orbit Plot");
            if (ImGui::BeginTable("##orbit_debug_plot", 2,
                                  ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                          ImGuiTableFlags_SizingStretchProp))
            {
                draw_debug_table_value("Lines active / pending", "%u / %u",
                                       plot_stats->active_line_count,
                                       plot_stats->pending_line_count);
                draw_debug_table_value("Segments depth / overlay", "%u / %u",
                                       plot_stats->depth_segment_count,
                                       plot_stats->overlay_segment_count);
                draw_debug_table_value("Solver segments base / planned", "%u / %u",
                                       perf.solver_segments_base,
                                       perf.solver_segments_planned);
                draw_debug_table_value("Pick segments before / after", "%u / %u",
                                       perf.pick_segments_before_cull,
                                       perf.pick_segments);
                draw_debug_table_value("Upload last / budget", "%.2f / %.2f MiB%s",
                                       upload_mib,
                                       budget_mib,
                                       plot_stats->upload_cap_hit_last_frame ? " [cap]" : "");
                draw_debug_table_value("Upload peak", "%.2f MiB, %.3f ms",
                                       peak_mib,
                                       plot_stats->upload_ms_peak);
                draw_debug_table_value("Timing solver / render / pick / upload", "%.3f / %.3f / %.3f / %.3f ms",
                                       perf.solver_ms_last,
                                       perf.render_lod_ms_last,
                                       perf.pick_lod_ms_last,
                                       plot_stats->upload_ms_last_frame);
                draw_debug_table_value("Cap hits render / pick / upload", "%llu / %llu / %llu",
                                       static_cast<unsigned long long>(perf.render_cap_hits_total),
                                       static_cast<unsigned long long>(perf.pick_cap_hits_total),
                                       static_cast<unsigned long long>(plot_stats->upload_cap_hits_total));
                draw_debug_table_value("Planned chunks drawn / total", "%u / %u",
                                       perf.planned_chunks_drawn,
                                       perf.planned_chunk_count);
                draw_debug_table_value("Fallback ranges", "%u", perf.planned_fallback_range_count);
                ImGui::EndTable();
            }
        }

        ImGui::SeparatorText("Update Flow");
        if (ImGui::BeginTable("##orbit_debug_update_flow", 2,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                      ImGuiTableFlags_SizingStretchProp))
        {
            draw_debug_table_value("Drag session / updates / requests", "%llu / %llu / %llu",
                                   static_cast<unsigned long long>(debug.drag_session_id),
                                   static_cast<unsigned long long>(debug.drag_update_count),
                                   static_cast<unsigned long long>(debug.request_count));
            draw_debug_table_value("Requests full / preview", "%llu / %llu",
                                   static_cast<unsigned long long>(debug.full_request_count),
                                   static_cast<unsigned long long>(debug.fast_preview_request_count));
            draw_debug_table_value("Solver / derived / publishes", "%llu / %llu / %llu",
                                   static_cast<unsigned long long>(debug.solver_result_count),
                                   static_cast<unsigned long long>(debug.derived_result_count),
                                   static_cast<unsigned long long>(debug.publish_count));
            draw_debug_table_value("Solver results full / preview", "%llu / %llu",
                                   static_cast<unsigned long long>(debug.full_solver_result_count),
                                   static_cast<unsigned long long>(debug.fast_preview_solver_result_count));
            draw_debug_table_age("Drag start", debug.drag_started_tp, now_tp);
            draw_debug_table_age("Last drag update", debug.last_drag_update_tp, now_tp);
            draw_debug_table_age("Last request", debug.last_request_tp, now_tp);
            draw_debug_table_age("Last solver result", debug.last_solver_result_tp, now_tp);
            draw_debug_table_age("Last derived apply", debug.last_derived_result_tp, now_tp);
            draw_debug_table_age("Last publish", debug.last_publish_tp, now_tp);
            if (PredictionDragDebugTelemetry::has_time(debug.last_drag_end_tp))
            {
                draw_debug_table_age("Last drag end", debug.last_drag_end_tp, now_tp);
            }
            ImGui::EndTable();
        }

        ImGui::SeparatorText("Latency");
        if (ImGui::BeginTable("##orbit_debug_latency", 2,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                      ImGuiTableFlags_SizingStretchProp))
        {
            draw_debug_table_value("Drag -> request", "%.3f ms (peak %.3f)",
                                   debug.drag_to_request_ms_last,
                                   debug.drag_to_request_ms_peak);
            draw_debug_table_value("Request -> solver", "%.3f ms (peak %.3f)",
                                   debug.request_to_solver_ms_last,
                                   debug.request_to_solver_ms_peak);
            draw_debug_table_value("Request -> derived", "%.3f ms (peak %.3f)",
                                   debug.request_to_derived_ms_last,
                                   debug.request_to_derived_ms_peak);
            draw_debug_table_value("Solver -> derived", "%.3f ms (peak %.3f)",
                                   debug.solver_to_derived_ms_last,
                                   debug.solver_to_derived_ms_peak);
            draw_debug_table_value("Drag apply", "%.3f ms (peak %.3f)",
                                   debug.drag_apply_ms_last,
                                   debug.drag_apply_ms_peak);
            draw_debug_table_value("Solver worker", "%.3f ms", active_track->solver_ms_last);
            draw_debug_table_value("Derived worker / frame / chunks", "%.3f / %.3f / %.3f ms",
                                   debug.derived_worker_ms_last,
                                   debug.derived_frame_build_ms_last,
                                   debug.derived_chunk_assembly_ms_last);
            draw_debug_table_value("Derived apply", "%.3f ms", debug.derived_apply_ms_last);
            draw_debug_table_value("Render LOD / chunk / fallback / pick", "%.3f / %.3f / %.3f / %.3f ms",
                                   _prediction->state().orbit_plot_perf.render_lod_ms_last,
                                   _prediction->state().orbit_plot_perf.planned_chunk_enqueue_ms_last,
                                   _prediction->state().orbit_plot_perf.planned_fallback_draw_ms_last,
                                   _prediction->state().orbit_plot_perf.pick_lod_ms_last);
            ImGui::EndTable();
        }

        ImGui::SeparatorText("Scale");
        if (ImGui::BeginTable("##orbit_debug_scale", 2,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                      ImGuiTableFlags_SizingStretchProp))
        {
            draw_debug_table_value("Chunk planned seg / samples", "%zu / %zu",
                                   debug.chunk_planned_segments_last,
                                   debug.chunk_planned_samples_last);
            draw_debug_table_value("Frame samples base / planned", "%zu / %zu",
                                   active_track->derived_diagnostics.frame_sample_count,
                                   active_track->derived_diagnostics.frame_sample_count_planned);
            draw_debug_table_value("Frame segments base / planned", "%zu / %zu",
                                   active_track->derived_diagnostics.frame_segment_count,
                                   active_track->derived_diagnostics.frame_segment_count_planned);
            draw_debug_table_value("Solver samples base / planned", "%zu / %zu",
                                   active_track->solver_diagnostics.trajectory_sample_count,
                                   active_track->solver_diagnostics.trajectory_sample_count_planned);
            if (_prediction->state().orbit_plot_perf.planned_window_valid ||
                !_maneuver.plan().nodes.empty())
            {
                const OrbitPlotPerfStats &perf = _prediction->state().orbit_plot_perf;
                draw_debug_table_bool("Planned window valid", perf.planned_window_valid);
                draw_debug_table_value("Planned window start / end", "%.3f / %.3f",
                                       perf.planned_window_t_start,
                                       perf.planned_window_t_end);
                draw_debug_table_value("Planned anchor / now", "%.3f / %.3f",
                                       perf.planned_window_anchor_s,
                                       perf.planned_window_now_s);
            }
            if (active_track->preview_anchor.valid)
            {
                draw_debug_table_value("Preview anchor node / time", "%d / %.3f",
                                       active_track->preview_anchor.anchor_node_id,
                                       active_track->preview_anchor.anchor_time_s);
                draw_debug_table_value("Preview windows req / visual / exact", "%.3f / %.3f / %.3f",
                                       active_track->preview_anchor.request_window_s,
                                       active_track->preview_anchor.visual_window_s,
                                       active_track->preview_anchor.exact_window_s);
            }
            ImGui::EndTable();
        }

        ImGui::Separator();
        if (drag_gate_remaining_ms > 0.0)
        {
            ImGui::TextWrapped(
                    "Throttle note: drag rebuild interval is currently limited by wall-time since the last preview request.");
        }
        if (!live_chunk_path_supported)
        {
            ImGui::TextWrapped(
                    "Chunk preview reuse is disabled in the current display frame, so live preview falls back to rebuilding derived display data.");
        }

        ImGui::End();
    }
} // namespace Game
