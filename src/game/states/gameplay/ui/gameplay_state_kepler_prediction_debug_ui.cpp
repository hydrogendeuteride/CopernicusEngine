#include "game/states/gameplay/gameplay_state.h"

#include "game/orbit/kepler/kepler_arc_info.h"
#include "game/states/gameplay/prediction_kepler/kepler_prediction_system.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>

namespace Game
{
    namespace
    {
        void draw_kepler_arc_info_table_value(const char *label, const char *fmt, ...)
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

        void draw_kepler_arc_info_table_bool(const char *label, const bool value)
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

        const KeplerPredictionState::Track *find_first_valid_kepler_track(
                const KeplerPredictionState &kepler)
        {
            for (const KeplerPredictionState::Track &track : kepler.tracks)
            {
                if (track.valid)
                {
                    return &track;
                }
            }
            return nullptr;
        }

    } // namespace

    void GameplayState::draw_kepler_orbit_debug_window(GameStateContext &ctx)
    {
        (void) ctx;
        if (!_kepler_prediction)
        {
            _show_kepler_orbit_debug = false;
            return;
        }

        const ImGuiViewport *viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(
                ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - 560.0f, viewport->WorkPos.y + 16.0f),
                ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(460.0f, 360.0f), ImGuiCond_FirstUseEver);

        bool window_open = _show_kepler_orbit_debug;
        if (!ImGui::Begin("Kepler Orbit Debug", &window_open))
        {
            _show_kepler_orbit_debug = window_open;
            ImGui::End();
            return;
        }
        _show_kepler_orbit_debug = window_open;

        bool display_changed = false;
        ImGui::SeparatorText("Draw");
        display_changed |= ImGui::Checkbox("Draw spacecraft Kepler tracks", &_kepler_draw_orbiter_tracks);
        display_changed |= ImGui::Checkbox("Draw celestial Kepler comparison tracks",
                                           &_kepler_draw_celestial_kepler_tracks);
        display_changed |= ImGui::Checkbox("Draw celestial N-body tracks", &_kepler_draw_celestial_nbody_tracks);
        if (display_changed)
        {
            mark_kepler_prediction_dirty();
        }

        bool quality_changed = false;
        ImGui::SeparatorText("Quality");
        float spacecraft_max_dt_s = static_cast<float>(_kepler_arc_line_options.max_time_step_s);
        if (ImGui::DragFloat("Spacecraft max dt (s)", &spacecraft_max_dt_s, 0.25f, 1.0f, 300.0f, "%.2f"))
        {
            _kepler_arc_line_options.max_time_step_s =
                    static_cast<double>(std::clamp(spacecraft_max_dt_s, 1.0f, 300.0f));
            quality_changed = true;
        }

        float spacecraft_max_chord_error_m =
                static_cast<float>(_kepler_arc_line_options.max_chord_error_m);
        if (ImGui::DragFloat("Spacecraft chord error (m)",
                             &spacecraft_max_chord_error_m,
                             10.0f,
                             0.0f,
                             1'000'000.0f,
                             "%.1f"))
        {
            _kepler_arc_line_options.max_chord_error_m =
                    static_cast<double>(std::clamp(spacecraft_max_chord_error_m,
                                                   0.0f,
                                                   1'000'000.0f));
            quality_changed = true;
        }

        int spacecraft_max_vertices_per_arc =
                static_cast<int>(std::min<std::size_t>(_kepler_arc_line_options.max_vertices_per_arc,
                                                       200000u));
        if (ImGui::DragInt("Spacecraft vertices / arc",
                           &spacecraft_max_vertices_per_arc,
                           64.0f,
                           64,
                           200000))
        {
            _kepler_arc_line_options.max_vertices_per_arc =
                    static_cast<std::size_t>(std::clamp(spacecraft_max_vertices_per_arc, 64, 200000));
            quality_changed = true;
        }

        int spacecraft_max_vertices_total =
                static_cast<int>(std::min<std::size_t>(_kepler_arc_line_options.max_vertices_total,
                                                       200000u));
        if (ImGui::DragInt("Spacecraft vertices total",
                           &spacecraft_max_vertices_total,
                           64.0f,
                           64,
                           200000))
        {
            _kepler_arc_line_options.max_vertices_total =
                    static_cast<std::size_t>(std::clamp(spacecraft_max_vertices_total, 64, 200000));
            quality_changed = true;
        }

        float open_orbit_horizon_h =
                static_cast<float>(_kepler_prediction_options.open_orbit_window_s / 3600.0);
        if (ImGui::DragFloat("Open orbit horizon (h)", &open_orbit_horizon_h, 0.25f, 0.25f, 720.0f, "%.2f"))
        {
            _kepler_prediction_options.open_orbit_window_s =
                    static_cast<double>(std::clamp(open_orbit_horizon_h, 0.25f, 720.0f)) * 3600.0;
            quality_changed = true;
        }

        float celestial_nbody_horizon_cap_h =
                static_cast<float>(_kepler_prediction_options.celestial_nbody_horizon_cap_s / 3600.0);
        if (ImGui::DragFloat("Celestial N-body cap (h)",
                             &celestial_nbody_horizon_cap_h,
                             0.25f,
                             0.0f,
                             720.0f,
                             "%.2f"))
        {
            _kepler_prediction_options.celestial_nbody_horizon_cap_s =
                    celestial_nbody_horizon_cap_h > 0.0f
                            ? static_cast<double>(std::clamp(celestial_nbody_horizon_cap_h,
                                                             0.25f,
                                                             720.0f)) * 3600.0
                            : 0.0;
            quality_changed = true;
        }

        if (quality_changed)
        {
            mark_kepler_prediction_dirty();
        }
        ImGui::Separator();

        const KeplerPredictionState &kepler = _kepler_prediction->state();
        const KeplerManeuverPlanState &maneuver_plan = _kepler_maneuver.plan();
        const KeplerManeuverNodeResolveResult &maneuver_resolve =
                _kepler_maneuver.last_node_resolve_result();
        const KeplerPredictionState::Track *active_track = find_active_player_kepler_track(kepler);
        const KeplerPredictionState::Track *summary_track =
                active_track ? active_track : find_first_valid_kepler_track(kepler);

        ImGui::SeparatorText("Maneuver");
        if (ImGui::BeginTable("##kepler_maneuver_summary", 2,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                      ImGuiTableFlags_SizingStretchProp))
        {
            draw_kepler_arc_info_table_value("Revision", "%llu",
                                             static_cast<unsigned long long>(_kepler_maneuver.revision()));
            draw_kepler_arc_info_table_value("Nodes", "%zu editor / %zu prediction",
                                             maneuver_plan.nodes.size(),
                                             _kepler_maneuver.prediction_nodes().size());
            draw_kepler_arc_info_table_value("Selected node", "%d", maneuver_plan.selected_node_id);
            if (const KeplerManeuverEditorNode *selected =
                    maneuver_plan.find_node(maneuver_plan.selected_node_id))
            {
                draw_kepler_arc_info_table_value("Selected DV RTN", "%.3f / %.3f / %.3f m/s",
                                                 selected->dv_rtn_mps.x,
                                                 selected->dv_rtn_mps.y,
                                                 selected->dv_rtn_mps.z);
            }
            else
            {
                draw_kepler_arc_info_table_value("Selected DV RTN", "%s", "none");
            }
            draw_kepler_arc_info_table_value("Display states", "%zu total / %zu valid",
                                             _kepler_maneuver.node_display_states().size(),
                                             maneuver_resolve.valid_node_count);
            if (!maneuver_plan.nodes.empty())
            {
                const auto [min_it, max_it] = std::minmax_element(
                        maneuver_plan.nodes.begin(),
                        maneuver_plan.nodes.end(),
                        [](const KeplerManeuverEditorNode &a, const KeplerManeuverEditorNode &b) {
                            return a.time_s < b.time_s;
                        });
                draw_kepler_arc_info_table_value("Node time range", "%.3f - %.3f s",
                                                 min_it->time_s,
                                                 max_it->time_s);
            }
            else
            {
                draw_kepler_arc_info_table_value("Node time range", "%s", "empty");
            }
            if (active_track)
            {
                draw_kepler_arc_info_table_value("Active track planned", "%zu arcs / %zu samples",
                                                 active_track->planned_arcs.size(),
                                                 active_track->planned_lines.vertices.size());
                draw_kepler_arc_info_table_value("Active track base end", "%.3f s",
                                                 active_track->orbit.base_arc.arc.t1_s);
            }
            else
            {
                draw_kepler_arc_info_table_value("Active track planned", "%s", "missing");
            }
            ImGui::EndTable();
        }

        ImGui::SeparatorText("Prediction");
        if (ImGui::BeginTable("##kepler_prediction_status", 2,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                      ImGuiTableFlags_SizingStretchProp))
        {
            draw_kepler_arc_info_table_bool("Enabled", kepler.enabled);
            draw_kepler_arc_info_table_bool("Valid", kepler.valid);
            draw_kepler_arc_info_table_bool("Dirty", kepler.dirty);
            draw_kepler_arc_info_table_value("Status", "%s", kepler_orbit_status_name(kepler.status));
            draw_kepler_arc_info_table_value("Revision", "%llu",
                                           static_cast<unsigned long long>(kepler.revision));
            draw_kepler_arc_info_table_value("Maneuver revision", "%llu",
                                           static_cast<unsigned long long>(kepler.maneuver_revision));
            draw_kepler_arc_info_table_value("Primary body", "%u",
                                           static_cast<unsigned int>(kepler.primary_body_id));
            draw_kepler_arc_info_table_value("World ref body", "%u",
                                          static_cast<unsigned int>(kepler.world_reference_body_id));
            draw_kepler_arc_info_table_value("Build time", "%.3f s", kepler.build_time_s);
            draw_kepler_arc_info_table_value("Horizon", "%.3f s", kepler.horizon_s);
            draw_kepler_arc_info_table_value(
                    "Spacecraft dt / chord / vertices", "%.3f s / %.1f m / %zu / %zu",
                    _kepler_arc_line_options.max_time_step_s,
                    _kepler_arc_line_options.max_chord_error_m,
                    _kepler_arc_line_options.max_vertices_per_arc,
                    _kepler_arc_line_options.max_vertices_total);
            draw_kepler_arc_info_table_value(
                    "Celestial line dt / vertices", "%.3f s / %zu",
                    _kepler_prediction_options.celestial_line_max_time_step_s,
                    _kepler_prediction_options.celestial_line_max_vertices_per_track);
            if (_kepler_prediction_options.celestial_nbody_horizon_cap_s > 0.0)
            {
                draw_kepler_arc_info_table_value(
                        "Celestial N-body cap", "%.3f s",
                        _kepler_prediction_options.celestial_nbody_horizon_cap_s);
            }
            else
            {
                draw_kepler_arc_info_table_value("Celestial N-body cap", "%s", "disabled");
            }
            draw_kepler_arc_info_table_value(
                    "Celestial ephem dt setting", "%.6f / %.3f s",
                    _kepler_prediction_options.celestial_nbody_ephemeris.min_dt_s,
                    _kepler_prediction_options.celestial_nbody_ephemeris.max_dt_s);
            draw_kepler_arc_info_table_bool("Celestial ephem valid",
                                         kepler.celestial_nbody_ephemeris.valid);
            draw_kepler_arc_info_table_value(
                    "Celestial ephem status", "%s",
                    kepler_orbit_status_name(kepler.celestial_nbody_ephemeris.status));
            draw_kepler_arc_info_table_value(
                    "Celestial ephem horizon", "src %.3f s / req %.3f s / built %.3f s",
                    kepler.celestial_nbody_ephemeris.uncapped_required_horizon_s,
                    kepler.celestial_nbody_ephemeris.required_horizon_s,
                    kepler.celestial_nbody_ephemeris.built_horizon_s);
            draw_kepler_arc_info_table_bool("Celestial horizon capped",
                                         kepler.celestial_nbody_ephemeris.horizon_capped);
            draw_kepler_arc_info_table_value(
                    "Celestial ephem window", "%.3f - %.3f s",
                    kepler.celestial_nbody_ephemeris.t0_s,
                    kepler.celestial_nbody_ephemeris.t_end_s);
            draw_kepler_arc_info_table_value(
                    "Celestial bodies / segments", "%zu / %zu",
                    kepler.celestial_nbody_ephemeris.body_count,
                    kepler.celestial_nbody_ephemeris.accepted_segments);
            draw_kepler_arc_info_table_value(
                    "Celestial ephem dt min/avg/max", "%.6f / %.6f / %.6f s",
                    kepler.celestial_nbody_ephemeris.min_dt_s,
                    kepler.celestial_nbody_ephemeris.avg_dt_s,
                    kepler.celestial_nbody_ephemeris.max_dt_s);
            draw_kepler_arc_info_table_value(
                    "Celestial ephem splits", "rejected %zu / forced %zu",
                    kepler.celestial_nbody_ephemeris.rejected_splits,
                    kepler.celestial_nbody_ephemeris.forced_boundary_splits);
            draw_kepler_arc_info_table_bool("Celestial ephem cap hit",
                                          kepler.celestial_nbody_ephemeris.hard_cap_hit);
            if (summary_track)
            {
                draw_kepler_arc_info_table_value("Base arcs / samples", "%zu / %zu",
                                              summary_track->base_arcs.size(),
                                              summary_track->base_lines.vertices.size());
                draw_kepler_arc_info_table_value("Planned arcs / samples", "%zu / %zu",
                                              summary_track->planned_arcs.size(),
                                              summary_track->planned_lines.vertices.size());
            }
            else
            {
                draw_kepler_arc_info_table_value("Base arcs / samples", "%s", "missing");
                draw_kepler_arc_info_table_value("Planned arcs / samples", "%s", "missing");
            }
            if (summary_track && summary_track->metrics.valid)
            {
                draw_kepler_arc_info_table_value("Regime", "%s",
                                              kepler_orbit_regime_name(summary_track->metrics.regime));
                draw_kepler_arc_info_table_value("Eccentricity", "%.6f", summary_track->metrics.eccentricity);
                draw_kepler_arc_info_table_value("Semi-major axis", "%.3f km",
                                              summary_track->metrics.semi_major_axis_m / 1000.0);
                draw_kepler_arc_info_table_value("Period", "%.3f s", summary_track->metrics.period_s);
            }
            ImGui::EndTable();
        }

        ImGui::End();
    }
} // namespace Game
