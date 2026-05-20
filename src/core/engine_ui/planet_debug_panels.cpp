#include "core/engine_ui/common.h"

namespace vk_engine::debug_ui
{
    void ui_planets(VulkanEngine *eng)
    {
        if (!eng || !eng->_sceneManager)
        {
            return;
        }

        SceneManager *scene = eng->_sceneManager.get();
        PlanetSystem *planets = scene->get_planet_system();
        if (!planets)
        {
            ImGui::TextUnformatted("Planet system not available");
            return;
        }

        bool enabled = planets->enabled();
        if (ImGui::Checkbox("Enable planet rendering", &enabled))
        {
            planets->set_enabled(enabled);
        }

        const WorldVec3 origin_world = scene->get_world_origin();
        const WorldVec3 cam_world = scene->getMainCamera().position_world;
        const glm::vec3 cam_local = scene->get_camera_local_position();

        ImGui::Separator();
        ImGui::Text("Camera world (m):  %.3f, %.3f, %.3f", cam_world.x, cam_world.y, cam_world.z);
        ImGui::Text("Camera local (m):  %.3f, %.3f, %.3f", cam_local.x, cam_local.y, cam_local.z);
        ImGui::Text("World origin (m):  %.3f, %.3f, %.3f", origin_world.x, origin_world.y, origin_world.z);

        auto look_at_world = [](Camera &cam, const WorldVec3 &target_world)
        {
            glm::dvec3 dirD = glm::normalize(target_world - cam.position_world);
            glm::vec3 dir = glm::normalize(glm::vec3(dirD));

            glm::vec3 up(0.0f, 1.0f, 0.0f);
            if (glm::length2(glm::cross(dir, up)) < 1e-6f)
            {
                up = glm::vec3(0.0f, 0.0f, 1.0f);
            }

            glm::vec3 f = dir;
            glm::vec3 r = glm::normalize(glm::cross(up, f));
            glm::vec3 u = glm::cross(f, r);

            glm::mat3 rot;
            rot[0] = r;
            rot[1] = u;
            rot[2] = -f; // -Z forward
            cam.orientation = glm::quat_cast(rot);
        };

        const PlanetSystem::PlanetBody *terrain_body = nullptr;
        for (const PlanetSystem::PlanetBody &b : planets->bodies())
        {
            if (b.terrain)
            {
                terrain_body = &b;
                break;
            }
        }

        ImGui::Separator();
        if (terrain_body)
        {
            const std::string label = "Terrain LOD / Perf (" + terrain_body->name + ")";
            if (ImGui::CollapsingHeader(label.c_str(), ImGuiTreeNodeFlags_DefaultOpen))
            {
                auto settings = planets->earth_quadtree_settings();
                bool changed = false;

                bool tint = planets->earth_debug_tint_patches_by_lod();
                if (ImGui::Checkbox("Debug: tint patches by LOD", &tint))
                {
                    planets->set_earth_debug_tint_patches_by_lod(tint);
                }

                int maxLevel = static_cast<int>(settings.max_level);
                if (ImGui::SliderInt("Max LOD level", &maxLevel, 0, 20))
                {
                    settings.max_level = static_cast<uint32_t>(std::max(0, maxLevel));
                    changed = true;
                }

                if (ImGui::SliderFloat("Target SSE (px)", &settings.target_sse_px, 4.0f, 128.0f, "%.1f"))
                {
                    settings.target_sse_px = std::max(settings.target_sse_px, 0.1f);
                    changed = true;
                }

                if (ImGui::SliderFloat("LOD hysteresis", &settings.lod_hysteresis_ratio, 0.0f, 0.75f, "%.2f"))
                {
                    settings.lod_hysteresis_ratio = std::clamp(settings.lod_hysteresis_ratio, 0.0f, 0.95f);
                    changed = true;
                }

                int maxPatches = static_cast<int>(settings.max_patches_visible);
                if (ImGui::SliderInt("Max visible patches", &maxPatches, 64, 20000))
                {
                    settings.max_patches_visible = static_cast<uint32_t>(std::max(6, maxPatches));
                    changed = true;
                }

                int createBudget = static_cast<int>(planets->earth_patch_create_budget_per_frame());
                if (ImGui::SliderInt("Patch create budget/frame", &createBudget, 0, 512))
                {
                    planets->set_earth_patch_create_budget_per_frame(static_cast<uint32_t>(std::max(0, createBudget)));
                }

                float createBudgetMs = planets->earth_patch_create_budget_ms();
                if (ImGui::DragFloat("Patch create budget (ms)", &createBudgetMs, 0.25f, 0.0f, 50.0f, "%.2f"))
                {
                    planets->set_earth_patch_create_budget_ms(std::max(0.0f, createBudgetMs));
                }

                int patchRes = static_cast<int>(planets->earth_patch_resolution());
                if (ImGui::SliderInt("Patch resolution (verts/edge)", &patchRes, 2, 129))
                {
                    planets->set_earth_patch_resolution(static_cast<uint32_t>(std::max(2, patchRes)));
                }

                int cacheMax = static_cast<int>(planets->earth_patch_cache_max());
                if (ImGui::SliderInt("Patch cache max", &cacheMax, 0, 50000))
                {
                    planets->set_earth_patch_cache_max(static_cast<uint32_t>(std::max(0, cacheMax)));
                }

                if (ImGui::Checkbox("Frustum cull", &settings.frustum_cull)) changed = true;
                if (ImGui::Checkbox("Horizon cull", &settings.horizon_cull)) changed = true;

                if (ImGui::Checkbox("RT guardrail (LOD floor)", &settings.rt_guardrail)) changed = true;
                if (settings.rt_guardrail)
                {
                    float maxEdge = static_cast<float>(settings.max_patch_edge_rt_m);
                    if (ImGui::DragFloat("RT max patch edge (m)", &maxEdge, 100.0f, 0.0f, 200000.0f, "%.0f"))
                    {
                        settings.max_patch_edge_rt_m = static_cast<double>(std::max(0.0f, maxEdge));
                        changed = true;
                    }

                    float maxAlt = static_cast<float>(settings.rt_guardrail_max_altitude_m);
                    if (ImGui::DragFloat("RT max altitude (m)", &maxAlt, 1000.0f, 0.0f, 2.0e6f, "%.0f"))
                    {
                        settings.rt_guardrail_max_altitude_m = static_cast<double>(std::max(0.0f, maxAlt));
                        changed = true;
                    }
                }

                if (changed)
                {
                    planets->set_earth_quadtree_settings(settings);
                }

                const PlanetSystem::EarthDebugStats &s = planets->terrain_debug_stats(terrain_body->name);
                const uint32_t effective_patch_resolution =
                        (terrain_body->patch_resolution_override != 0u)
                                ? terrain_body->patch_resolution_override
                                : planets->earth_patch_resolution();
                const float effective_target_sse =
                        (terrain_body->target_sse_px_override > 0.0f)
                                ? terrain_body->target_sse_px_override
                                : planets->earth_quadtree_settings().target_sse_px;
                ImGui::Separator();
                ImGui::Text("Effective patch resolution: %u%s",
                            effective_patch_resolution,
                            terrain_body->patch_resolution_override != 0u ? " (body override)" : " (global)");
                ImGui::Text("Effective target SSE: %.1f px%s",
                            effective_target_sse,
                            terrain_body->target_sse_px_override > 0.0f ? " (body override)" : " (global)");
                ImGui::Text("Visible patches: %u  (rendered: %u | est. tris: %u)",
                            s.visible_patches,
                            s.rendered_patches,
                            s.estimated_triangles);
                ImGui::Text("Cache size: %u  (created this frame: %u)", s.patch_cache_size, s.created_patches);
                ImGui::Text("Quadtree: max level used %u | visited %u | culled %u | budget-limited %u",
                            s.quadtree.max_level_used,
                            s.quadtree.nodes_visited,
                            s.quadtree.nodes_culled,
                            s.quadtree.splits_budget_limited);
                ImGui::Text("CPU ms: quadtree %.2f | create %.2f | emit %.2f | total %.2f",
                            s.ms_quadtree, s.ms_patch_create, s.ms_emit, s.ms_total);
            }
        }
        else
        {
            ImGui::TextUnformatted("No terrain planet active");
        }

        ImGui::Separator();
        if (ImGui::CollapsingHeader("Planet Tools", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (ImGui::Button("Clear all planets"))
            {
                planets->clear_planets(true);
            }

            static int selected_planet = 0;
            static std::string edit_target_name;
            static bool edit_visible = true;
            static bool edit_terrain = false;
            static double edit_center[3] = {0.0, 0.0, 0.0};
            static double edit_radius_km = 1.0;

            const auto &bodies = planets->bodies();
            if (selected_planet < 0) selected_planet = 0;
            if (!bodies.empty() && selected_planet >= static_cast<int>(bodies.size()))
            {
                selected_planet = static_cast<int>(bodies.size()) - 1;
            }

            if (ImGui::BeginListBox("Planets"))
            {
                for (int i = 0; i < static_cast<int>(bodies.size()); ++i)
                {
                    const PlanetSystem::PlanetBody &b = bodies[i];
                    const bool is_selected = (selected_planet == i);
                    const char *tag = b.terrain ? " (terrain)" : " (mesh)";
                    const std::string label = b.name + tag;
                    if (ImGui::Selectable(label.c_str(), is_selected))
                    {
                        selected_planet = i;
                    }
                }
                ImGui::EndListBox();
            }

            if (!bodies.empty())
            {
                const PlanetSystem::PlanetBody &b = bodies[static_cast<size_t>(selected_planet)];
                if (edit_target_name != b.name)
                {
                    edit_target_name = b.name;
                    edit_visible = b.visible;
                    edit_terrain = b.terrain;
                    edit_center[0] = b.center_world.x;
                    edit_center[1] = b.center_world.y;
                    edit_center[2] = b.center_world.z;
                    edit_radius_km = b.radius_m / 1000.0;
                }

                ImGui::Text("Selected: %s", b.name.c_str());

                if (ImGui::Checkbox("Visible##selected", &edit_visible))
                {
                    planets->set_planet_visible(edit_target_name, edit_visible);
                }
                if (ImGui::Checkbox("Terrain##selected", &edit_terrain))
                {
                    planets->set_planet_terrain(edit_target_name, edit_terrain);
                }

                if (ImGui::InputScalarN("Center (world m)##selected", ImGuiDataType_Double, edit_center, 3, nullptr, nullptr, "%.3f"))
                {
                    planets->set_planet_center(edit_target_name, WorldVec3(edit_center[0], edit_center[1], edit_center[2]));
                }
                if (ImGui::DragScalar("Radius (km)##selected", ImGuiDataType_Double, &edit_radius_km, 10.0f, nullptr, nullptr, "%.3f"))
                {
                    planets->set_planet_radius(edit_target_name, std::max(1.0, edit_radius_km * 1000.0));
                }

                const WorldVec3 to_cam = cam_world - b.center_world;
                const double dist = glm::length(to_cam);
                const double alt_base_radius_m = dist - b.radius_m;
                const WorldVec3 dir_from_center =
                        (dist > 1.0e-12) ? (to_cam * (1.0 / dist)) : WorldVec3(0.0, 0.0, 1.0);
                const double terrain_displacement_m =
                        (b.terrain && b.visible && b.radius_m > 0.0)
                                ? planets->sample_terrain_displacement_m(b, dir_from_center)
                                : 0.0;
                const double clearance_above_terrain_m = alt_base_radius_m - terrain_displacement_m;

                ImGui::Text("Altitude above base radius: %.3f km", alt_base_radius_m / 1000.0);
                if (b.terrain)
                {
                    ImGui::Text("Terrain displacement at camera: %.3f km", terrain_displacement_m / 1000.0);
                    ImGui::Text("Clearance above terrain: %.3f km", clearance_above_terrain_m / 1000.0);
                }
                else
                {
                    ImGui::TextUnformatted("Terrain displacement at camera: n/a");
                    ImGui::TextUnformatted("Clearance above terrain: n/a");
                }

                if (ImGui::Button("Look at##selected"))
                {
                    look_at_world(scene->getMainCamera(), b.center_world);
                }
                ImGui::SameLine();
                if (ImGui::Button("Teleport: 10000 km above##selected"))
                {
                    scene->getMainCamera().position_world =
                        b.center_world + WorldVec3(0.0, 0.0, b.radius_m + 1.0e7);
                    look_at_world(scene->getMainCamera(), b.center_world);
                }

                if (ImGui::Button("Teleport: 1000 km orbit##selected"))
                {
                    scene->getMainCamera().position_world =
                        b.center_world + WorldVec3(0.0, 0.0, b.radius_m + 1.0e6);
                    look_at_world(scene->getMainCamera(), b.center_world);
                }
                ImGui::SameLine();
                if (ImGui::Button("Teleport: 10 km above##selected"))
                {
                    scene->getMainCamera().position_world =
                        b.center_world + WorldVec3(0.0, 0.0, b.radius_m + 1.0e4);
                    look_at_world(scene->getMainCamera(), b.center_world);
                }

                if (ImGui::Button("Destroy selected"))
                {
                    planets->destroy_planet(edit_target_name);
                    edit_target_name.clear();
                    selected_planet = 0;
                }
            }

            ImGui::Separator();
            ImGui::TextUnformatted("Create mesh planet");

            static char new_name[64] = "Mars";
            static double new_center[3] = {50000000.0, 0.0, 0.0};
            static double new_radius_km = 3390.0;
            static float new_color[4] = {0.8f, 0.35f, 0.25f, 1.0f};
            static float new_metallic = 0.0f;
            static float new_roughness = 1.0f;
            static int new_sectors = 48;
            static int new_stacks = 24;

            ImGui::InputText("Name", new_name, IM_ARRAYSIZE(new_name));
            ImGui::InputScalarN("Center (world m)", ImGuiDataType_Double, new_center, 3, nullptr, nullptr, "%.3f");
            ImGui::DragScalar("Radius (km)", ImGuiDataType_Double, &new_radius_km, 10.0f, nullptr, nullptr, "%.3f");
            ImGui::ColorEdit4("Base color", new_color);
            ImGui::DragFloat("Metallic", &new_metallic, 0.01f, 0.0f, 1.0f, "%.2f");
            ImGui::DragFloat("Roughness", &new_roughness, 0.01f, 0.0f, 1.0f, "%.2f");
            ImGui::SliderInt("Sectors", &new_sectors, 8, 256);
            ImGui::SliderInt("Stacks", &new_stacks, 4, 256);

            if (ImGui::Button("Create"))
            {
                PlanetSystem::MeshPlanetCreateInfo info{};
                info.name = new_name;
                info.center_world = WorldVec3(new_center[0], new_center[1], new_center[2]);
                info.radius_m = std::max(1.0, new_radius_km * 1000.0);
                info.visible = true;
                info.base_color = glm::vec4(new_color[0], new_color[1], new_color[2], new_color[3]);
                info.metallic = std::clamp(new_metallic, 0.0f, 1.0f);
                info.roughness = std::clamp(new_roughness, 0.0f, 1.0f);
                info.sectors = static_cast<uint32_t>(std::max(3, new_sectors));
                info.stacks = static_cast<uint32_t>(std::max(2, new_stacks));
                planets->create_mesh_planet(info);
            }

            ImGui::Separator();
            ImGui::TextUnformatted("Create terrain planet");

            static char terr_name[64] = "Earth2";
            static double terr_center[3] = {0.0, 0.0, 0.0};
            static double terr_radius_km = 6378.137;
            static float terr_color[4] = {1.0f, 1.0f, 1.0f, 1.0f};
            static float terr_metallic = 0.0f;
            static float terr_roughness = 1.0f;
            static char terr_albedo_dir[256] = "planets/earth/albedo/L0";

            ImGui::InputText("Name##terrain_create", terr_name, IM_ARRAYSIZE(terr_name));
            ImGui::InputScalarN("Center (world m)##terrain_create", ImGuiDataType_Double, terr_center, 3, nullptr, nullptr, "%.3f");
            ImGui::DragScalar("Radius (km)##terrain_create", ImGuiDataType_Double, &terr_radius_km, 10.0f, nullptr, nullptr, "%.3f");
            ImGui::ColorEdit4("Base color##terrain_create", terr_color);
            ImGui::DragFloat("Metallic##terrain_create", &terr_metallic, 0.01f, 0.0f, 1.0f, "%.2f");
            ImGui::DragFloat("Roughness##terrain_create", &terr_roughness, 0.01f, 0.0f, 1.0f, "%.2f");
            ImGui::InputText("Albedo dir##terrain_create", terr_albedo_dir, IM_ARRAYSIZE(terr_albedo_dir));

            if (ImGui::Button("Create##terrain_create"))
            {
                PlanetSystem::TerrainPlanetCreateInfo info{};
                info.name = terr_name;
                info.center_world = WorldVec3(terr_center[0], terr_center[1], terr_center[2]);
                info.radius_m = std::max(1.0, terr_radius_km * 1000.0);
                info.visible = true;
                info.base_color = glm::vec4(terr_color[0], terr_color[1], terr_color[2], terr_color[3]);
                info.metallic = std::clamp(terr_metallic, 0.0f, 1.0f);
                info.roughness = std::clamp(terr_roughness, 0.0f, 1.0f);
                info.albedo_dir = terr_albedo_dir;
                planets->create_terrain_planet(info);
            }
        }
    }

    void ui_debug_draw(VulkanEngine *eng)
    {
        if (!eng || !eng->_context)
        {
            ImGui::TextUnformatted("Engine context not available");
            return;
        }

        DebugDrawSystem *dd = eng->_debugDraw.get();
        if (dd)
        {
            ImGui::SeparatorText("Debug Draw");

            bool enabled = dd->settings().enabled;
            if (ImGui::Checkbox("Enabled", &enabled))
            {
                dd->settings().enabled = enabled;
            }

            ImGui::SameLine();
            ImGui::Checkbox("Depth-tested", &dd->settings().show_depth_tested);
            ImGui::SameLine();
            ImGui::Checkbox("Overlay", &dd->settings().show_overlay);

            bool physics_layer = (dd->settings().layer_mask & static_cast<uint32_t>(DebugDrawLayer::Physics)) != 0u;
            if (ImGui::Checkbox("Layer: Physics", &physics_layer))
            {
                if (physics_layer)
                {
                    dd->settings().layer_mask |= static_cast<uint32_t>(DebugDrawLayer::Physics);
                }
                else
                {
                    dd->settings().layer_mask &= ~static_cast<uint32_t>(DebugDrawLayer::Physics);
                }
            }

            ImGui::SliderInt("Segments", &dd->settings().segments, 8, 128);
            ImGui::SliderFloat("Line width (px)", &dd->settings().line_width_px, 1.0f, 8.0f, "%.1f");
            ImGui::SliderFloat("Line AA (px)", &dd->settings().line_aa_px, 0.0f, 4.0f, "%.1f");
        }
    }
} // namespace vk_engine::debug_ui
