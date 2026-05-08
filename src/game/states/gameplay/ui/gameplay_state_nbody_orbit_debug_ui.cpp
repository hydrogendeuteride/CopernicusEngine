#include "game/states/gameplay/gameplay_state.h"
#include "game/orbit/nbody/tuning.h"
#include "game/states/gameplay/maneuver_nbody/gameplay_state_maneuver_gizmo_helpers.h"
#include "game/states/gameplay/prediction_nbody/gameplay_prediction_adapter.h"
#include "core/debug_draw/debug_draw.h"
#include "core/engine.h"
#include "core/game_api.h"
#include "core/orbit_plot/orbit_plot.h"

#include "imgui.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <string>

namespace Game
{
    namespace Gizmo = ManeuverGizmoHelpers;

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

        const char *selected_frame_label(const GameplayPredictionState &state)
        {
            if (state.frame_selection.selected_index >= 0 &&
                state.frame_selection.selected_index < static_cast<int>(state.frame_selection.options.size()))
            {
                return state.frame_selection.options[static_cast<std::size_t>(state.frame_selection.selected_index)]
                        .label.c_str();
            }
            return "Unknown";
        }

        const char *selected_analysis_label(const GameplayPredictionState &state)
        {
            if (state.analysis_selection.selected_index >= 0 &&
                state.analysis_selection.selected_index < static_cast<int>(state.analysis_selection.options.size()))
            {
                return state.analysis_selection.options[static_cast<std::size_t>(state.analysis_selection.selected_index)]
                        .label.c_str();
            }
            return "Unknown";
        }

        bool same_analysis_spec(const PredictionAnalysisSpec &a, const PredictionAnalysisSpec &b)
        {
            return a.mode == b.mode && a.fixed_body_id == b.fixed_body_id;
        }

        void draw_nbody_frame_controls(GameplayPredictionAdapter &prediction, GameplayPredictionState &state)
        {
            if (ImGui::BeginCombo("Display frame", selected_frame_label(state)))
            {
                for (std::size_t i = 0; i < state.frame_selection.options.size(); ++i)
                {
                    const PredictionFrameOption &option = state.frame_selection.options[i];
                    const bool selected = PredictionFrameResolver::same_frame_spec(
                            option.spec,
                            state.frame_selection.spec);
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

            if (ImGui::BeginCombo("Analysis frame", selected_analysis_label(state)))
            {
                for (std::size_t i = 0; i < state.analysis_selection.options.size(); ++i)
                {
                    const PredictionAnalysisOption &option = state.analysis_selection.options[i];
                    const bool selected = same_analysis_spec(option.spec, state.analysis_selection.spec);
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

            if (ImGui::Button("Reset display frame"))
            {
                (void) prediction.set_prediction_frame_spec(prediction.default_prediction_frame_spec());
            }
            ImGui::SameLine();
            if (ImGui::Button("Reset analysis"))
            {
                (void) prediction.set_prediction_analysis_spec(PredictionAnalysisSpec{});
            }
        }

    } // anonymous namespace

    void GameplayState::draw_nbody_orbit_debug_window(GameStateContext &ctx)
    {
        GameplayPredictionAdapter prediction(build_prediction_access());
        OrbitPlotSystem *orbit_plot =
                (ctx.renderer && ctx.renderer->_context) ? ctx.renderer->_context->orbit_plot : nullptr;
        const OrbitPlotSystem::Stats *plot_stats = orbit_plot ? &orbit_plot->stats() : nullptr;

        const ImGuiViewport *viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(
                ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - 560.0f, viewport->WorkPos.y + 16.0f),
                ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(620.0f, 560.0f), ImGuiCond_FirstUseEver);

        bool window_open = _show_nbody_orbit_debug;
        if (!ImGui::Begin("N Body Orbit Debug", &window_open))
        {
            _show_nbody_orbit_debug = window_open;
            ImGui::End();
            return;
        }
        _show_nbody_orbit_debug = window_open;

        prediction.rebuild_prediction_subjects();
        prediction.rebuild_prediction_frame_options();
        prediction.rebuild_prediction_analysis_options();

        const PredictionTrackState *active_track = prediction.active_prediction_track();
        const auto now_tp = PredictionDragDebugTelemetry::Clock::now();
        const bool have_active_track = active_track != nullptr;
        const std::string track_label =
                active_track ? prediction.prediction_subject_label(active_track->key) : std::string("None");

        const bool have_sim_now = _orbit.scenario_owner() != nullptr;
        const double sim_now_s = have_sim_now ? _orbit.scenario_owner()->sim.time_s() : 0.0;
        const bool have_build_time = active_track && active_track->cache.identity.valid && have_sim_now;
        const double sim_since_build_s =
                have_build_time ? std::max(0.0, sim_now_s - active_track->cache.identity.build_time_s) : 0.0;

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

        const bool active_track_with_maneuvers =
                active_track &&
                active_track->supports_maneuvers &&
                _maneuver.settings().nodes_enabled &&
                !_maneuver.plan().nodes.empty();
        const bool live_preview_effective = _maneuver.live_preview_active(active_track_with_maneuvers);

        double drag_gate_remaining_ms = 0.0;
        bool live_chunk_path_supported = true;
        if (active_track)
        {
            const PredictionDragDebugTelemetry &debug = active_track->drag_debug;
            drag_gate_remaining_ms =
                    PredictionDragDebugTelemetry::has_time(debug.last_request_tp)
                            ? std::max(0.0,
                                       OrbitPredictionTuning::kDragRebuildMinIntervalS -
                                               std::chrono::duration<double>(
                                                       now_tp - debug.last_request_tp).count()) *
                                      1000.0
                            : 0.0;

            const orbitsim::TrajectoryFrameSpec frame_spec =
                    active_track->cache.display.resolved_frame_spec_valid
                            ? active_track->cache.display.resolved_frame_spec
                            : _prediction->state().frame_selection.spec;
            live_chunk_path_supported =
                    frame_spec.type != orbitsim::TrajectoryFrameType::Inertial &&
                    frame_spec.type != orbitsim::TrajectoryFrameType::LVLH;
        }

        if (ImGui::BeginTable("##nbody_orbit_debug_status", 2,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                      ImGuiTableFlags_SizingStretchProp))
        {
            draw_debug_table_value("Track", "%s", track_label.c_str());
            if (active_track)
            {
                draw_debug_table_value("Display frame", "%s", selected_frame_label(_prediction->state()));
                draw_debug_table_value("Analysis frame", "%s", selected_analysis_label(_prediction->state()));
                draw_debug_table_value("Cache generation", "%llu visible / %llu authoritative",
                                       static_cast<unsigned long long>(
                                               active_track->cache.identity.generation_id),
                                       static_cast<unsigned long long>(
                                               active_track->authoritative_cache.identity.generation_id));
            }
            ImGui::EndTable();
        }

        if (!ImGui::BeginTabBar("##nbody_orbit_debug_tabs"))
        {
            ImGui::End();
            return;
        }

        if (ImGui::BeginTabItem("Frames"))
        {
            draw_nbody_frame_controls(prediction, _prediction->state());
            if (ImGui::BeginTable("##orbit_debug_frame_selection", 2,
                                  ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                          ImGuiTableFlags_SizingStretchProp))
            {
                draw_debug_table_value("Display frame", "%s", selected_frame_label(_prediction->state()));
                draw_debug_table_value("Analysis frame", "%s", selected_analysis_label(_prediction->state()));
                draw_debug_table_value("Display revision", "%llu",
                                       static_cast<unsigned long long>(_prediction->state().display_frame_revision));
                if (active_track)
                {
                    draw_debug_table_bool("Resolved frame valid",
                                          active_track->cache.display.resolved_frame_spec_valid);
                    draw_debug_table_bool("Live chunk path", live_chunk_path_supported);
                }
                ImGui::EndTable();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Draw"))
        {
            ImGui::SeparatorText("Prediction Lines");
            ImGui::Checkbox("Prediction full orbit", &_prediction->state().draw_full_orbit);
            ImGui::Checkbox("Prediction future segment", &_prediction->state().draw_future_segment);
            ImGui::Checkbox("Velocity ray", &_prediction->state().draw_velocity_ray);
            ImGui::SameLine();
            ImGui::Checkbox("Planned dashed", &_prediction->state().draw_config.draw_planned_as_dashed);
            ImGui::SameLine();
            ImGui::BeginDisabled(!_maneuver.settings().nodes_enabled);
            ImGui::Checkbox("Node overlay", &_maneuver.settings().nodes_debug_draw);
            ImGui::EndDisabled();

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

            ImGui::SeparatorText("Budgets");
            float render_error_px = static_cast<float>(_prediction->budget().render_error_px);
            if (ImGui::DragFloat("Render error (px)", &render_error_px, 0.01f, 0.05f, 4.0f, "%.2f"))
            {
                _prediction->budget().render_error_px = std::clamp(static_cast<double>(render_error_px), 0.05, 4.0);
            }

            int render_max_segments_cpu = _prediction->budget().render_max_segments_cpu;
            if (ImGui::DragInt("Render max segments (CPU)", &render_max_segments_cpu, 50.0f, 64, 200000))
            {
                _prediction->budget().render_max_segments_cpu =
                        std::clamp(render_max_segments_cpu, 64, 200000);
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

            ImGui::SeparatorText("Draw Layers");
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
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Solver"))
        {
            ImGui::SeparatorText("Policy");
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

            float orbiter_min_window_s =
                    static_cast<float>(_prediction->state().sampling_policy.orbiter_min_window_s);
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

            float celestial_min_window_s =
                    static_cast<float>(_prediction->state().sampling_policy.celestial_min_window_s);
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

            if (!have_active_track)
            {
                ImGui::SeparatorText("Runtime");
                ImGui::TextUnformatted("No active prediction track.");
                ImGui::EndTabItem();
            }
            else
            {
                const PredictionDragDebugTelemetry &debug = active_track->drag_debug;
                ImGui::SeparatorText("Runtime");
                if (ImGui::BeginTable("##orbit_debug_prediction_runtime", 2,
                                      ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                              ImGuiTableFlags_SizingStretchProp))
                {
                    draw_debug_table_value("Track", "%s", track_label.c_str());
                    draw_debug_table_value("Result quality", "%s",
                                           prediction_solve_quality_label(debug.last_result_solve_quality));
                    draw_debug_table_value("Last request quality", "%s",
                                           prediction_solve_quality_label(debug.last_request_solve_quality));
                    draw_debug_table_value("Last request window / impulses", "%.3f s / %zu",
                                           debug.last_request_future_window_s,
                                           debug.last_request_maneuver_count);
                    draw_debug_table_bool("Last request preview patch", debug.last_request_preview_patch);
                    draw_debug_table_bool("Last request full stream", debug.last_request_full_stream_publish);
                    draw_debug_table_value("Cache generation", "%llu visible / %llu authoritative",
                                           static_cast<unsigned long long>(
                                                   active_track->cache.identity.generation_id),
                                           static_cast<unsigned long long>(
                                                   active_track->authoritative_cache.identity.generation_id));
                    draw_debug_table_bool("Visible cache valid", active_track->cache.identity.valid);
                    draw_debug_table_bool("Authoritative cache valid", active_track->authoritative_cache.identity.valid);
                    if (have_build_time)
                    {
                        draw_debug_table_value("Sim since cache build", "%.3f s", sim_since_build_s);
                    }
                    draw_debug_table_bool("Supports maneuvers", active_track->supports_maneuvers);
                    ImGui::EndTable();
                }

                ImGui::SeparatorText("Stages");
                const OrbitPredictionDiagnostics &solver_diag = active_track->solver_diagnostics;
                const OrbitPredictionDerivedDiagnostics &derived_diag = active_track->derived_diagnostics;
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
                draw_prediction_stage_diag("Frame base", derived_diag.frame_base, derived_diag.frame_sample_count);
                if (derived_diag.frame_planned.accepted_segments > 0 ||
                    derived_diag.frame_sample_count_planned > 0)
                {
                    draw_prediction_stage_diag(
                            "Frame planned",
                            derived_diag.frame_planned,
                            derived_diag.frame_sample_count_planned);
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
                    ImGui::EndTable();
                }
                ImGui::EndTabItem();
            }
        }

        if (ImGui::BeginTabItem("Maneuver"))
        {
            ImGui::SeparatorText("Plan");
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
                draw_debug_table_value("Preview window", "%.0f s",
                                       _maneuver.settings().plan_windows.preview_window_s);
                draw_debug_table_bool("Warp to node active", _maneuver.runtime().warp_to_time_active);
                draw_debug_table_value("Armed execute node", "%d", _maneuver.runtime().execute_node_id);
                ImGui::EndTable();
            }

            if (_maneuver.gizmo_interaction().node_id >= 0)
            {
                ImGui::Text("Gizmo node/axis: %d / %s",
                            _maneuver.gizmo_interaction().node_id,
                            Gizmo::maneuver_axis_label(
                                    _maneuver.settings().gizmo_basis_mode,
                                    _maneuver.gizmo_interaction().axis));
            }

            if (!have_active_track)
            {
                ImGui::SeparatorText("Telemetry");
                ImGui::TextUnformatted("No active prediction track.");
                ImGui::EndTabItem();
            }
            else
            {
                const PredictionDragDebugTelemetry &debug = active_track->drag_debug;
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
                    ImGui::EndTable();
                }

                if (active_track->preview_anchor.valid)
                {
                    ImGui::SeparatorText("Preview Anchor");
                    if (ImGui::BeginTable("##orbit_debug_preview_anchor", 2,
                                          ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                                  ImGuiTableFlags_SizingStretchProp))
                    {
                        draw_debug_table_value("Node / time", "%d / %.3f",
                                               active_track->preview_anchor.anchor_node_id,
                                               active_track->preview_anchor.anchor_time_s);
                        draw_debug_table_value("Windows req / visual / exact", "%.3f / %.3f / %.3f",
                                               active_track->preview_anchor.request_window_s,
                                               active_track->preview_anchor.visual_window_s,
                                               active_track->preview_anchor.exact_window_s);
                        ImGui::EndTable();
                    }
                }

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
                ImGui::EndTabItem();
            }
        }

        if (ImGui::BeginTabItem("Plot"))
        {
            if (!plot_stats)
            {
                ImGui::TextDisabled("OrbitPlotSystem unavailable.");
            }
            else
            {
                const double upload_mib =
                        static_cast<double>(plot_stats->upload_bytes_last_frame) / (1024.0 * 1024.0);
                const double budget_mib =
                        static_cast<double>(plot_stats->upload_budget_bytes) / (1024.0 * 1024.0);
                const double peak_mib =
                        static_cast<double>(plot_stats->upload_bytes_peak) / (1024.0 * 1024.0);
                const OrbitPlotPerfStats &perf = _prediction->state().orbit_plot_perf;

                ImGui::SeparatorText("Render & Upload");
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
                    draw_debug_table_value("Pick segments before / after", "%u / %u",
                                           perf.pick_segments_before_cull,
                                           perf.pick_segments);
                    draw_debug_table_value("Planned chunks drawn / total", "%u / %u",
                                           perf.planned_chunks_drawn,
                                           perf.planned_chunk_count);
                    draw_debug_table_value("Fallback ranges", "%u", perf.planned_fallback_range_count);
                    draw_debug_table_value("Upload last / budget", "%.2f / %.2f MiB%s",
                                           upload_mib,
                                           budget_mib,
                                           plot_stats->upload_cap_hit_last_frame ? " [cap]" : "");
                    draw_debug_table_value("Upload peak", "%.2f MiB, %.3f ms",
                                           peak_mib,
                                           plot_stats->upload_ms_peak);
                    draw_debug_table_value("Timing render / pick / upload", "%.3f / %.3f / %.3f ms",
                                           perf.render_lod_ms_last,
                                           perf.pick_lod_ms_last,
                                           plot_stats->upload_ms_last_frame);
                    draw_debug_table_value("Cap hits render / pick / upload", "%llu / %llu / %llu",
                                           static_cast<unsigned long long>(perf.render_cap_hits_total),
                                           static_cast<unsigned long long>(perf.pick_cap_hits_total),
                                           static_cast<unsigned long long>(plot_stats->upload_cap_hits_total));
                    if (perf.planned_window_valid || !_maneuver.plan().nodes.empty())
                    {
                        draw_debug_table_bool("Planned window valid", perf.planned_window_valid);
                        draw_debug_table_value("Planned window start / end", "%.3f / %.3f",
                                               perf.planned_window_t_start,
                                               perf.planned_window_t_end);
                        draw_debug_table_value("Planned anchor / now", "%.3f / %.3f",
                                               perf.planned_window_anchor_s,
                                               perf.planned_window_now_s);
                    }
                    ImGui::EndTable();
                }
            }
            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
        ImGui::End();
    }
} // namespace Game
