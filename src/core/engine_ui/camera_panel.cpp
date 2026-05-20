#include "core/engine_ui/common.h"

namespace vk_engine::debug_ui
{
    void ui_overview(VulkanEngine *eng)
    {
        if (!eng) return;
        float fps = (eng->stats.frametime > 0.0f) ? (1000.0f / eng->stats.frametime) : 0.0f;
        ImGui::Text("frametime %.2f ms (%.1f FPS)", eng->stats.frametime, fps);
        ImGui::Text("draw time %.2f ms", eng->stats.mesh_draw_time);
        ImGui::Text("update time %.2f ms", eng->_sceneManager->stats.scene_update_time);
        ImGui::Text("triangles %i", eng->stats.triangle_count);
        ImGui::Text("draws %i", eng->stats.drawcall_count);

        ImGui::Separator();
        ImGui::Text("Draw extent: %ux%u", eng->_drawExtent.width, eng->_drawExtent.height);
        auto scExt = eng->_swapchainManager->swapchainExtent();
        ImGui::Text("Swapchain:   %ux%u", scExt.width, scExt.height);
        ImGui::Text("Draw fmt:    %s", string_VkFormat(eng->_swapchainManager->drawImage().imageFormat));
        ImGui::Text("Swap fmt:    %s", string_VkFormat(eng->_swapchainManager->swapchainImageFormat()));

        if (eng->_sceneManager)
        {
            ImGui::Separator();
            WorldVec3 origin = eng->_sceneManager->get_world_origin();
            WorldVec3 camWorld = eng->_sceneManager->getMainCamera().position_world;
            glm::vec3 camLocal = eng->_sceneManager->get_camera_local_position();
            ImGui::Text("Origin (world): (%.3f, %.3f, %.3f)", origin.x, origin.y, origin.z);
            ImGui::Text("Camera (world): (%.3f, %.3f, %.3f)", camWorld.x, camWorld.y, camWorld.z);
            ImGui::Text("Camera (local): (%.3f, %.3f, %.3f)", camLocal.x, camLocal.y, camLocal.z);
        }
    }

    void ui_camera(VulkanEngine *eng)
    {
        if (!eng || !eng->_sceneManager)
        {
            ImGui::TextUnformatted("SceneManager not available");
            return;
        }

        SceneManager *sceneMgr = eng->_sceneManager.get();
        CameraRig &rig = sceneMgr->getCameraRig();
        Camera &cam = sceneMgr->getMainCamera();

        // Mode switch
        static const char *k_mode_names[] = {"Free", "Orbit", "Follow", "Chase", "Fixed"};
        int mode = static_cast<int>(rig.mode());
        if (ImGui::Combo("Mode", &mode, k_mode_names, IM_ARRAYSIZE(k_mode_names)))
        {
            rig.set_mode(static_cast<CameraMode>(mode), *sceneMgr, cam);
            if (eng->_input)
            {
                eng->_input->set_cursor_mode(CursorMode::Normal);
            }
        }

        ImGui::Text("Active mode: %s", rig.mode_name());
        ImGui::Separator();

        // Camera state (world)
        double pos[3] = {cam.position_world.x, cam.position_world.y, cam.position_world.z};
        if (ImGui::InputScalarN("Position (world)", ImGuiDataType_Double, pos, 3, nullptr, nullptr, "%.3f"))
        {
            cam.position_world = WorldVec3(pos[0], pos[1], pos[2]);
        }
        float fov = cam.fovDegrees;
        if (ImGui::SliderFloat("FOV (deg)", &fov, 30.0f, 110.0f))
        {
            cam.fovDegrees = fov;
        }

        WorldVec3 origin = sceneMgr->get_world_origin();
        glm::vec3 camLocal = sceneMgr->get_camera_local_position();
        ImGui::Text("Origin (world): (%.3f, %.3f, %.3f)", origin.x, origin.y, origin.z);
        ImGui::Text("Camera (local): (%.3f, %.3f, %.3f)", camLocal.x, camLocal.y, camLocal.z);

        bool terrain_clamp_enabled = rig.terrain_surface_clamp_enabled();
        if (ImGui::Checkbox("Clamp camera above terrain", &terrain_clamp_enabled))
        {
            rig.set_terrain_surface_clamp_enabled(terrain_clamp_enabled);
        }

        ImGui::BeginDisabled(!terrain_clamp_enabled);
        double terrain_clearance_m = rig.terrain_surface_clearance_m();
        if (ImGui::InputDouble("Terrain surface clearance (m)", &terrain_clearance_m, 0.05, 0.5, "%.3f"))
        {
            rig.set_terrain_surface_clearance_m(terrain_clearance_m);
        }
        ImGui::EndDisabled();

        auto target_from_last_pick = [&](CameraTarget &target) -> bool {
            PickingSystem *picking = eng->picking();
            if (!picking) return false;
            const auto &pick = picking->last_pick();
            if (!pick.valid) return false;

            if (pick.ownerType == RenderObject::OwnerType::MeshInstance)
            {
                // Many procedural objects (planets etc.) tag draws as "MeshInstance" for picking,
                // but they don't exist in SceneManager::dynamicMeshInstances. Only use a
                // MeshInstance camera target if it resolves.
                WorldVec3 t{};
                glm::quat r{};
                glm::vec3 s{};
                if (sceneMgr->getMeshInstanceTRSWorld(pick.ownerName, t, r, s))
                {
                    target.type = CameraTargetType::MeshInstance;
                    target.name = pick.ownerName;
                }
                else if (PlanetSystem *planets = sceneMgr->get_planet_system())
                {
                    if (PlanetSystem::PlanetBody *body = planets->find_body_by_name(pick.ownerName))
                    {
                        target.type = CameraTargetType::MeshInstance;
                        target.name = body->name;
                    }
                    else
                    {
                        target.type = CameraTargetType::WorldPoint;
                        target.world_point = pick.worldPos;
                        target.name.clear();
                    }
                }
                else
                {
                    target.type = CameraTargetType::WorldPoint;
                    target.world_point = pick.worldPos;
                    target.name.clear();
                }
            }
            else if (pick.ownerType == RenderObject::OwnerType::GLTFInstance)
            {
                target.type = CameraTargetType::GLTFInstance;
                target.name = pick.ownerName;
            }
            else
            {
                target.type = CameraTargetType::WorldPoint;
                target.world_point = pick.worldPos;
                target.name.clear();
            }
            return true;
        };

        auto draw_target = [&](const char *id, CameraTarget &target, char *name_buf, size_t name_buf_size) {
            ImGui::PushID(id);
            static const char *k_target_types[] = {"None", "WorldPoint", "MeshInstance", "GLTFInstance"};
            int type = static_cast<int>(target.type);
            if (ImGui::Combo("Target type", &type, k_target_types, IM_ARRAYSIZE(k_target_types)))
            {
                target.type = static_cast<CameraTargetType>(type);
                if (target.type != CameraTargetType::MeshInstance && target.type != CameraTargetType::GLTFInstance)
                {
                    target.name.clear();
                    if (name_buf_size > 0)
                    {
                        name_buf[0] = '\0';
                    }
                }
            }

            if (target.type == CameraTargetType::WorldPoint)
            {
                double p[3] = {target.world_point.x, target.world_point.y, target.world_point.z};
                if (ImGui::InputScalarN("World point", ImGuiDataType_Double, p, 3, nullptr, nullptr, "%.3f"))
                {
                    target.world_point = WorldVec3(p[0], p[1], p[2]);
                }
            }
            else if (target.type == CameraTargetType::MeshInstance || target.type == CameraTargetType::GLTFInstance)
            {
                if (std::strncmp(name_buf, target.name.c_str(), name_buf_size) != 0)
                {
                    std::snprintf(name_buf, name_buf_size, "%s", target.name.c_str());
                }
                ImGui::InputText("Target name", name_buf, name_buf_size);
                target.name = name_buf;
            }

            float local_offset[3] = {target.local_offset.x, target.local_offset.y, target.local_offset.z};
            if (ImGui::InputFloat3("Local offset", local_offset))
            {
                target.local_offset = glm::vec3(local_offset[0], local_offset[1], local_offset[2]);
            }

            WorldVec3 tpos{};
            glm::quat trot{};
            bool ok = rig.resolve_target(*sceneMgr, target, tpos, trot);
            ImGui::Text("Resolved: %s", ok ? "yes" : "no");
            if (ok)
            {
                ImGui::Text("Target world: (%.3f, %.3f, %.3f)", tpos.x, tpos.y, tpos.z);
            }
            ImGui::PopID();
        };

        // Free
        if (ImGui::CollapsingHeader("Free", ImGuiTreeNodeFlags_DefaultOpen))
        {
            auto &s = rig.free_settings();
            ImGui::InputFloat("Move speed (u/s)", &s.move_speed);
            s.move_speed = std::clamp(s.move_speed, 0.06f, 300.0f);
            ImGui::InputFloat("Look sensitivity", &s.look_sensitivity);
            ImGui::InputFloat("Roll speed (rad/s)", &s.roll_speed);
            ImGui::TextUnformatted("Roll keys: Q/E");
        }

        // Orbit
        if (ImGui::CollapsingHeader("Orbit"))
        {
            auto &s = rig.orbit_settings();
            static char orbitName[128] = "";
            draw_target("orbit_target", s.target, orbitName, IM_ARRAYSIZE(orbitName));
            if (ImGui::Button("Orbit target = Last Pick"))
            {
                target_from_last_pick(s.target);
            }
            ImGui::InputDouble("Distance", &s.distance, 0.1, 1.0, "%.3f");
            s.distance = std::clamp(s.distance,
                                    OrbitCameraSettings::kMinDistance,
                                    OrbitCameraSettings::kMaxDistance);
            float yawDeg = glm::degrees(s.yaw);
            float pitchDeg = glm::degrees(s.pitch);
            if (ImGui::SliderFloat("Yaw (deg)", &yawDeg, -180.0f, 180.0f))
            {
                s.yaw = glm::radians(yawDeg);
            }
            if (ImGui::SliderFloat("Pitch (deg)", &pitchDeg, -89.0f, 89.0f))
            {
                s.pitch = glm::radians(pitchDeg);
            }
            ImGui::InputFloat("Look sensitivity##orbit", &s.look_sensitivity);

            ImGui::Separator();
            ImGui::Text("Reference Up Vector");
            ImGui::InputFloat3("Up##orbit_up", &s.reference_up.x);
            if (ImGui::Button("Normalize Up"))
            {
                rig.set_orbit_reference_up(s.reference_up);
            }
            ImGui::SameLine();
            if (ImGui::Button("Reset to World Y"))
            {
                rig.set_orbit_reference_up(glm::vec3(0.0f, 1.0f, 0.0f));
            }
            if (ImGui::Button("Align Up to Target"))
            {
                rig.align_orbit_up_to_target();
            }
        }

        // Follow
        if (ImGui::CollapsingHeader("Follow"))
        {
            auto &s = rig.follow_settings();
            static char followName[128] = "";
            draw_target("follow_target", s.target, followName, IM_ARRAYSIZE(followName));
            if (ImGui::Button("Follow target = Last Pick"))
            {
                target_from_last_pick(s.target);
            }
            ImGui::InputFloat3("Position offset (local)", &s.position_offset_local.x);

            glm::vec3 rotDeg = glm::degrees(glm::eulerAngles(s.rotation_offset));
            float r[3] = {rotDeg.x, rotDeg.y, rotDeg.z};
            if (ImGui::InputFloat3("Rotation offset (deg XYZ)", r))
            {
                glm::mat4 R = glm::eulerAngleXYZ(glm::radians(r[0]), glm::radians(r[1]), glm::radians(r[2]));
                s.rotation_offset = glm::quat_cast(R);
            }
            if (ImGui::Button("Reset rotation offset"))
            {
                s.rotation_offset = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            }
        }

        // Chase
        if (ImGui::CollapsingHeader("Chase"))
        {
            auto &s = rig.chase_settings();
            static char chaseName[128] = "";
            draw_target("chase_target", s.target, chaseName, IM_ARRAYSIZE(chaseName));
            if (ImGui::Button("Chase target = Last Pick"))
            {
                target_from_last_pick(s.target);
            }
            ImGui::InputFloat3("Position offset (local)##chase", &s.position_offset_local.x);

            glm::vec3 rotDeg = glm::degrees(glm::eulerAngles(s.rotation_offset));
            float r[3] = {rotDeg.x, rotDeg.y, rotDeg.z};
            if (ImGui::InputFloat3("Rotation offset (deg XYZ)##chase", r))
            {
                glm::mat4 R = glm::eulerAngleXYZ(glm::radians(r[0]), glm::radians(r[1]), glm::radians(r[2]));
                s.rotation_offset = glm::quat_cast(R);
            }

            ImGui::SliderFloat("Position lag (1/s)", &s.position_lag, 0.0f, 30.0f);
            ImGui::SliderFloat("Rotation lag (1/s)", &s.rotation_lag, 0.0f, 30.0f);
        }

        // Fixed
        if (ImGui::CollapsingHeader("Fixed"))
        {
            ImGui::TextUnformatted("Fixed mode does not modify the camera automatically.");
        }
    }
} // namespace vk_engine::debug_ui
