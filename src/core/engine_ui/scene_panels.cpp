#include "core/engine_ui/common.h"

namespace vk_engine::debug_ui
{
    void ui_scene_editor(VulkanEngine *eng)
    {
        if (!eng || !eng->_sceneManager)
        {
            ImGui::TextUnformatted("SceneManager not available");
            return;
        }

        PickingSystem *picking = eng->picking();

        // Spawn glTF instances (runtime)
        ImGui::TextUnformatted("Spawn glTF instance");
        static char gltfPath[256] = "mirage_c/scene.gltf";
        static char gltfName[128] = "gltf_01";
        static float gltfPos[3] = {0.0f, 0.0f, 0.0f};
        static float gltfRot[3] = {0.0f, 0.0f, 0.0f}; // pitch, yaw, roll (deg)
        static float gltfScale[3] = {1.0f, 1.0f, 1.0f};
        ImGui::InputText("Model path (assets/models/...)", gltfPath, IM_ARRAYSIZE(gltfPath));
        ImGui::InputText("Instance name", gltfName, IM_ARRAYSIZE(gltfName));
        ImGui::InputFloat3("Position", gltfPos);
        ImGui::InputFloat3("Rotation (deg XYZ)", gltfRot);
        ImGui::InputFloat3("Scale", gltfScale);
        if (ImGui::Button("Add glTF instance"))
        {
            glm::mat4 T = glm::translate(glm::mat4(1.0f), glm::vec3(gltfPos[0], gltfPos[1], gltfPos[2]));
            glm::mat4 R = glm::eulerAngleXYZ(glm::radians(gltfRot[0]),
                                             glm::radians(gltfRot[1]),
                                             glm::radians(gltfRot[2]));
            glm::mat4 S = glm::scale(glm::mat4(1.0f), glm::vec3(gltfScale[0], gltfScale[1], gltfScale[2]));
            glm::mat4 M = T * R * S;
            eng->addGLTFInstance(gltfName, gltfPath, M);
        }

        ImGui::Separator();
        // Spawn primitive mesh instances (cube/sphere)
        ImGui::TextUnformatted("Spawn primitive");
        static int primType = 0; // 0 = cube, 1 = sphere
        static char primName[128] = "prim_01";
        static float primPos[3] = {0.0f, 0.0f, 0.0f};
        static float primRot[3] = {0.0f, 0.0f, 0.0f};
        static float primScale[3] = {1.0f, 1.0f, 1.0f};
        ImGui::RadioButton("Cube", &primType, 0); ImGui::SameLine();
        ImGui::RadioButton("Sphere", &primType, 1);
        ImGui::InputText("Primitive name", primName, IM_ARRAYSIZE(primName));
        ImGui::InputFloat3("Prim Position", primPos);
        ImGui::InputFloat3("Prim Rotation (deg XYZ)", primRot);
        ImGui::InputFloat3("Prim Scale", primScale);
        if (ImGui::Button("Add primitive instance"))
        {
            std::shared_ptr<MeshAsset> mesh = (primType == 0) ? eng->cubeMesh : eng->sphereMesh;
            if (mesh)
            {
                glm::mat4 T = glm::translate(glm::mat4(1.0f), glm::vec3(primPos[0], primPos[1], primPos[2]));
                glm::mat4 R = glm::eulerAngleXYZ(glm::radians(primRot[0]),
                                                 glm::radians(primRot[1]),
                                                 glm::radians(primRot[2]));
                glm::mat4 S = glm::scale(glm::mat4(1.0f), glm::vec3(primScale[0], primScale[1], primScale[2]));
                glm::mat4 M = T * R * S;
                eng->_sceneManager->addMeshInstance(primName, mesh, M);
            }
        }

        ImGui::Separator();
        // Delete selected model/primitive (uses last pick if valid, otherwise hover)
        static std::string deleteStatus;
        if (ImGui::Button("Delete selected"))
        {
            deleteStatus.clear();
            const PickingSystem::PickInfo *pick = nullptr;
            if (picking)
            {
                const auto &last = picking->last_pick();
                const auto &hover = picking->hover_pick();
                pick = last.valid ? &last : (hover.valid ? &hover : nullptr);
            }
            if (!pick || pick->ownerName.empty())
            {
                deleteStatus = "No selection to delete.";
            }
            else if (pick->ownerType == RenderObject::OwnerType::MeshInstance)
            {
                bool ok = eng->_sceneManager->removeMeshInstance(pick->ownerName);
                if (ok && picking)
                {
                    picking->clear_owner_picks(RenderObject::OwnerType::MeshInstance, pick->ownerName);
                    picking->clear_owner_binding(RenderObject::OwnerType::MeshInstance, pick->ownerName);
                }
                deleteStatus = ok ? "Removed mesh instance: " + pick->ownerName
                                  : "Mesh instance not found: " + pick->ownerName;
            }
            else if (pick->ownerType == RenderObject::OwnerType::GLTFInstance)
            {
                bool ok = eng->_sceneManager->removeGLTFInstance(pick->ownerName);
                if (ok)
                {
                    deleteStatus = "Removed glTF instance: " + pick->ownerName;
                    if (picking)
                    {
                        picking->clear_owner_picks(RenderObject::OwnerType::GLTFInstance, pick->ownerName);
                        picking->clear_owner_binding(RenderObject::OwnerType::GLTFInstance, pick->ownerName);
                    }
                }
                else
                {
                    deleteStatus = "glTF instance not found: " + pick->ownerName;
                }
            }
            else
            {
                deleteStatus = "Cannot delete this object type (static scene).";
            }
        }
        if (!deleteStatus.empty())
        {
            ImGui::TextUnformatted(deleteStatus.c_str());
        }
    }

    // Lights editor (Point + Spot lights)
    void ui_lights(VulkanEngine *eng)
    {
        if (!eng || !eng->_sceneManager)
        {
            ImGui::TextUnformatted("SceneManager not available");
            return;
        }

        SceneManager *sceneMgr = eng->_sceneManager.get();

        // Sunlight editor (Directional light)
        ImGui::TextUnformatted("Sunlight (directional)");
        {
            // Shaders consume a normalized -sceneData.sunlightDirection.xyz.
            // Expose Lsun directly in the UI (direction *towards* the sun).
            glm::vec3 Lsun = -sceneMgr->getSunlightDirection();
            const float len2 = glm::length2(Lsun);
            Lsun = (len2 > 1.0e-8f) ? glm::normalize(Lsun) : glm::vec3(0.0f, 1.0f, 0.0f);

            float dir[3] = {Lsun.x, Lsun.y, Lsun.z};
            if (ImGui::InputFloat3("Direction (to sun)##sun_dir", dir, "%.3f"))
            {
                glm::vec3 v{dir[0], dir[1], dir[2]};
                const float vlen2 = glm::length2(v);
                v = (vlen2 > 1.0e-8f) ? glm::normalize(v) : glm::vec3(0.0f, 1.0f, 0.0f);
                sceneMgr->setSunlightDirection(-v);
            }

            if (ImGui::Button("Reset##sun_reset"))
            {
                sceneMgr->setSunlightDirection(glm::vec3(-0.2f, -1.0f, -0.3f));
            }
        }

        ImGui::Separator();

        // Point light editor
        ImGui::TextUnformatted("Point lights");
            const auto &lights = sceneMgr->getPointLights();
            ImGui::Text("Active lights: %zu", lights.size());

            static int selectedLight = -1;
            if (selectedLight >= static_cast<int>(lights.size()))
            {
                selectedLight = static_cast<int>(lights.size()) - 1;
            }

            if (ImGui::BeginListBox("Light list"))
            {
                for (size_t i = 0; i < lights.size(); ++i)
                {
                    std::string label = fmt::format("Light {}", i);
                    const bool isSelected = (selectedLight == static_cast<int>(i));
                    if (ImGui::Selectable(label.c_str(), isSelected))
                    {
                        selectedLight = static_cast<int>(i);
                    }
                }
                ImGui::EndListBox();
            }

            // Controls for the selected light
            if (selectedLight >= 0 && selectedLight < static_cast<int>(lights.size()))
            {
                SceneManager::PointLight pl{};
                if (sceneMgr->getPointLight(static_cast<size_t>(selectedLight), pl))
                {
                    double pos[3] = {pl.position_world.x, pl.position_world.y, pl.position_world.z};
                    float col[3] = {pl.color.r, pl.color.g, pl.color.b};
                    bool changed = false;

                    changed |= ImGui::InputScalarN("Position (world)", ImGuiDataType_Double, pos, 3, nullptr, nullptr, "%.3f");
                    changed |= ImGui::SliderFloat("Radius", &pl.radius, 0.1f, 1000.0f);
                    changed |= ImGui::ColorEdit3("Color", col);
                    changed |= ImGui::SliderFloat("Intensity", &pl.intensity, 0.0f, 100.0f);
                    changed |= ImGui::Checkbox("Cast shadows##point_cast", &pl.cast_shadows);

                    if (changed)
                    {
                        pl.position_world = WorldVec3(pos[0], pos[1], pos[2]);
                        pl.color = glm::vec3(col[0], col[1], col[2]);
                        sceneMgr->setPointLight(static_cast<size_t>(selectedLight), pl);
                    }

                    if (ImGui::Button("Remove selected light"))
                    {
                        sceneMgr->removePointLight(static_cast<size_t>(selectedLight));
                        selectedLight = -1;
                    }
                }
            }

            // Controls for adding a new light
            ImGui::Separator();
            ImGui::TextUnformatted("Add point light");
            static double newPos[3] = {0.0, 1.0, 0.0};
            static float newRadius = 10.0f;
            static float newColor[3] = {1.0f, 1.0f, 1.0f};
            static float newIntensity = 5.0f;
            static bool newCastShadows = true;

            ImGui::InputScalarN("New position (world)", ImGuiDataType_Double, newPos, 3, nullptr, nullptr, "%.3f");
            ImGui::SliderFloat("New radius", &newRadius, 0.1f, 1000.0f);
            ImGui::ColorEdit3("New color", newColor);
            ImGui::SliderFloat("New intensity", &newIntensity, 0.0f, 100.0f);
            ImGui::Checkbox("New cast shadows##point_new_cast", &newCastShadows);

            if (ImGui::Button("Add point light"))
            {
                SceneManager::PointLight pl{};
                pl.position_world = WorldVec3(newPos[0], newPos[1], newPos[2]);
                pl.radius = newRadius;
                pl.color = glm::vec3(newColor[0], newColor[1], newColor[2]);
                pl.intensity = newIntensity;
                pl.cast_shadows = newCastShadows;

                const size_t oldCount = sceneMgr->getPointLightCount();
                sceneMgr->addPointLight(pl);
                selectedLight = static_cast<int>(oldCount);
            }

            if (ImGui::Button("Clear all lights"))
            {
                sceneMgr->clearPointLights();
                selectedLight = -1;
            }

            // Spot light editor
            ImGui::Separator();
            ImGui::TextUnformatted("Spot lights");

            const auto &spotLights = sceneMgr->getSpotLights();
            ImGui::Text("Active spot lights: %zu", spotLights.size());

            static int selectedSpot = -1;
            if (selectedSpot >= static_cast<int>(spotLights.size()))
            {
                selectedSpot = static_cast<int>(spotLights.size()) - 1;
            }

            if (ImGui::BeginListBox("Spot light list##spot_list"))
            {
                for (size_t i = 0; i < spotLights.size(); ++i)
                {
                    std::string label = fmt::format("Spot {}", i);
                    const bool isSelected = (selectedSpot == static_cast<int>(i));
                    if (ImGui::Selectable(label.c_str(), isSelected))
                    {
                        selectedSpot = static_cast<int>(i);
                    }
                }
                ImGui::EndListBox();
            }

            if (selectedSpot >= 0 && selectedSpot < static_cast<int>(spotLights.size()))
            {
                SceneManager::SpotLight sl{};
                if (sceneMgr->getSpotLight(static_cast<size_t>(selectedSpot), sl))
                {
                    double pos[3] = {sl.position_world.x, sl.position_world.y, sl.position_world.z};
                    float dir[3] = {sl.direction.x, sl.direction.y, sl.direction.z};
                    float col[3] = {sl.color.r, sl.color.g, sl.color.b};
                    bool changed = false;

                    changed |= ImGui::InputScalarN("Position (world)##spot_pos", ImGuiDataType_Double, pos, 3, nullptr, nullptr, "%.3f");
                    changed |= ImGui::InputFloat3("Direction##spot_dir", dir, "%.3f");
                    changed |= ImGui::SliderFloat("Radius##spot_radius", &sl.radius, 0.1f, 1000.0f);
                    changed |= ImGui::SliderFloat("Inner angle (deg)##spot_inner", &sl.inner_angle_deg, 0.0f, 89.0f);
                    changed |= ImGui::SliderFloat("Outer angle (deg)##spot_outer", &sl.outer_angle_deg, 0.0f, 89.9f);
                    changed |= ImGui::ColorEdit3("Color##spot_color", col);
                    changed |= ImGui::SliderFloat("Intensity##spot_intensity", &sl.intensity, 0.0f, 100.0f);
                    changed |= ImGui::Checkbox("Cast shadows##spot_cast", &sl.cast_shadows);

                    if (changed)
                    {
                        sl.position_world = WorldVec3(pos[0], pos[1], pos[2]);
                        glm::vec3 d{dir[0], dir[1], dir[2]};
                        sl.direction = (glm::length(d) > 1.0e-6f) ? glm::normalize(d) : glm::vec3(0.0f, -1.0f, 0.0f);
                        sl.color = glm::vec3(col[0], col[1], col[2]);
                        sl.inner_angle_deg = std::clamp(sl.inner_angle_deg, 0.0f, 89.0f);
                        sl.outer_angle_deg = std::clamp(sl.outer_angle_deg, sl.inner_angle_deg, 89.9f);
                        sceneMgr->setSpotLight(static_cast<size_t>(selectedSpot), sl);
                    }

                    if (ImGui::Button("Remove selected spot light##spot_remove"))
                    {
                        sceneMgr->removeSpotLight(static_cast<size_t>(selectedSpot));
                        selectedSpot = -1;
                    }
                }
            }

            ImGui::Separator();
            ImGui::TextUnformatted("Add spot light");
            static double newSpotPos[3] = {0.0, 2.0, 0.0};
            static float newSpotDir[3] = {0.0f, -1.0f, 0.0f};
            static float newSpotRadius = 10.0f;
            static float newSpotInner = 15.0f;
            static float newSpotOuter = 25.0f;
            static float newSpotColor[3] = {1.0f, 1.0f, 1.0f};
            static float newSpotIntensity = 10.0f;
            static bool newSpotCastShadows = true;

            ImGui::InputScalarN("New position (world)##spot_new_pos", ImGuiDataType_Double, newSpotPos, 3, nullptr, nullptr, "%.3f");
            ImGui::InputFloat3("New direction##spot_new_dir", newSpotDir, "%.3f");
            ImGui::SliderFloat("New radius##spot_new_radius", &newSpotRadius, 0.1f, 1000.0f);
            ImGui::SliderFloat("New inner angle (deg)##spot_new_inner", &newSpotInner, 0.0f, 89.0f);
            ImGui::SliderFloat("New outer angle (deg)##spot_new_outer", &newSpotOuter, 0.0f, 89.9f);
            if (newSpotInner > newSpotOuter)
            {
                newSpotOuter = newSpotInner;
            }
            ImGui::ColorEdit3("New color##spot_new_color", newSpotColor);
            ImGui::SliderFloat("New intensity##spot_new_intensity", &newSpotIntensity, 0.0f, 100.0f);
            ImGui::Checkbox("New cast shadows##spot_new_cast", &newSpotCastShadows);

            if (ImGui::Button("Add spot light##spot_add"))
            {
                SceneManager::SpotLight sl{};
                sl.position_world = WorldVec3(newSpotPos[0], newSpotPos[1], newSpotPos[2]);
                glm::vec3 d{newSpotDir[0], newSpotDir[1], newSpotDir[2]};
                sl.direction = (glm::length(d) > 1.0e-6f) ? glm::normalize(d) : glm::vec3(0.0f, -1.0f, 0.0f);
                sl.radius = newSpotRadius;
                sl.color = glm::vec3(newSpotColor[0], newSpotColor[1], newSpotColor[2]);
                sl.intensity = newSpotIntensity;
                sl.inner_angle_deg = std::clamp(newSpotInner, 0.0f, 89.0f);
                sl.outer_angle_deg = std::clamp(newSpotOuter, sl.inner_angle_deg, 89.9f);
                sl.cast_shadows = newSpotCastShadows;

                const size_t oldCount = sceneMgr->getSpotLightCount();
                sceneMgr->addSpotLight(sl);
                selectedSpot = static_cast<int>(oldCount);
            }

            if (ImGui::Button("Clear all spot lights##spot_clear"))
            {
                sceneMgr->clearSpotLights();
                selectedSpot = -1;
            }
    }

    // Picking & Gizmo - picking info and transform editor
    void ui_picking_gizmo(VulkanEngine *eng)
    {
        if (!eng || !eng->_sceneManager)
        {
            ImGui::TextUnformatted("SceneManager not available");
            return;
        }

        SceneManager *sceneMgr = eng->_sceneManager.get();
        PickingSystem *picking = eng->picking();
        auto selection_level_label = [](PickingSystem::SelectionLevel level) -> const char * {
            switch (level)
            {
                case PickingSystem::SelectionLevel::Object: return "object";
                case PickingSystem::SelectionLevel::Member: return "member";
                case PickingSystem::SelectionLevel::Node: return "node";
                case PickingSystem::SelectionLevel::Primitive: return "primitive";
                case PickingSystem::SelectionLevel::None:
                default: return "none";
            }
        };

        // Last pick info
        if (picking && picking->last_pick().valid)
        {
            const auto &last = picking->last_pick();
            const char *meshName = last.mesh ? last.mesh->name.c_str() : "<unknown>";
            const char *sceneName = "<none>";
            if (last.scene && !last.scene->debugName.empty())
            {
                sceneName = last.scene->debugName.c_str();
            }
            ImGui::Text("Last pick scene: %s", sceneName);
            ImGui::Text("Last pick source: %s",
                        picking->use_id_buffer_picking() ? "ID buffer" : "CPU raycast");
            ImGui::Text("Last pick object ID: %u", picking->last_pick_object_id());
            ImGui::Text("Last pick mesh: %s (surface %u)", meshName, last.surfaceIndex);
            ImGui::Text("World pos: (%.3f, %.3f, %.3f)",
                        last.worldPos.x,
                        last.worldPos.y,
                        last.worldPos.z);
            const char *ownerTypeStr = "none";
            switch (last.ownerType)
            {
                case RenderObject::OwnerType::MeshInstance: ownerTypeStr = "mesh instance"; break;
                case RenderObject::OwnerType::GLTFInstance: ownerTypeStr = "glTF instance"; break;
                default: break;
            }
            const char *ownerName = last.ownerName.empty() ? "<unnamed>" : last.ownerName.c_str();
            ImGui::Text("Owner: %s (%s)", ownerName, ownerTypeStr);
            const char *objectName = last.objectName.empty() ? "<unnamed>" : last.objectName.c_str();
            const char *memberName = last.memberName.empty() ? "<unnamed>" : last.memberName.c_str();
            ImGui::Text("Logical object: %s", objectName);
            ImGui::Text("Logical member: %s", memberName);
            ImGui::Text("Selection level: %s", selection_level_label(last.selectionLevel));

            const bool can_select_object = !last.objectName.empty() &&
                                           last.selectionLevel != PickingSystem::SelectionLevel::Object;
            const bool can_select_member = !last.memberName.empty() &&
                                           last.selectionLevel != PickingSystem::SelectionLevel::Member;
            const bool can_select_primitive = last.kind == PickingSystem::PickInfo::Kind::SceneObject &&
                                              last.selectionLevel != PickingSystem::SelectionLevel::Primitive;

            if (!can_select_object)
            {
                ImGui::BeginDisabled();
            }
            if (ImGui::Button("Select logical object") && can_select_object)
            {
                (void)picking->select_last_pick_object();
            }
            if (!can_select_object)
            {
                ImGui::EndDisabled();
            }

            ImGui::SameLine();
            if (!can_select_member)
            {
                ImGui::BeginDisabled();
            }
            if (ImGui::Button("Select concrete member") && can_select_member)
            {
                (void)picking->select_last_pick_member();
            }
            if (!can_select_member)
            {
                ImGui::EndDisabled();
            }

            ImGui::SameLine();
            if (!can_select_primitive)
            {
                ImGui::BeginDisabled();
            }
            if (ImGui::Button("Select primitive") && can_select_primitive)
            {
                (void)picking->set_last_pick_selection_level(PickingSystem::SelectionLevel::Primitive);
            }
            if (!can_select_primitive)
            {
                ImGui::EndDisabled();
            }

            ImGui::Text("Indices: first=%u count=%u",
                        last.firstIndex,
                        last.indexCount);

            if (eng->_sceneManager)
            {
                const SceneManager::PickingDebug &dbg = eng->_sceneManager->getPickingDebug();
                ImGui::Text("Mesh BVH used: %s, hit: %s, fallback box: %s",
                            dbg.usedMeshBVH ? "yes" : "no",
                            dbg.meshBVHHit ? "yes" : "no",
                            dbg.meshBVHFallbackBox ? "yes" : "no");
                if (dbg.meshBVHPrimCount > 0)
                {
                    ImGui::Text("Mesh BVH stats: prims=%u, nodes=%u",
                                dbg.meshBVHPrimCount,
                                dbg.meshBVHNodeCount);
                }
            }
        }
        else
        {
            ImGui::TextUnformatted("Last pick: <none>");
        }
        ImGui::Separator();
        if (picking && picking->hover_pick().valid)
        {
            const auto &hover = picking->hover_pick();
            const char *meshName = hover.mesh ? hover.mesh->name.c_str() : "<unknown>";
            ImGui::Text("Hover mesh: %s (surface %u)", meshName, hover.surfaceIndex);
            const char *ownerTypeStr = "none";
            switch (hover.ownerType)
            {
                case RenderObject::OwnerType::MeshInstance: ownerTypeStr = "mesh instance"; break;
                case RenderObject::OwnerType::GLTFInstance: ownerTypeStr = "glTF instance"; break;
                default: break;
            }
            const char *ownerName = hover.ownerName.empty() ? "<unnamed>" : hover.ownerName.c_str();
            ImGui::Text("Hover owner: %s (%s)", ownerName, ownerTypeStr);
            const char *objectName = hover.objectName.empty() ? "<unnamed>" : hover.objectName.c_str();
            const char *memberName = hover.memberName.empty() ? "<unnamed>" : hover.memberName.c_str();
            ImGui::Text("Hover object/member: %s / %s", objectName, memberName);
            ImGui::Text("Hover selection level: %s", selection_level_label(hover.selectionLevel));
        }
        else
        {
            ImGui::TextUnformatted("Hover: <none>");
        }
        if (picking && !picking->drag_selection().empty())
        {
            ImGui::Text("Drag selection: %zu objects", picking->drag_selection().size());
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Object Gizmo (ImGuizmo)");

        // Choose a pick to edit: prefer last pick, then hover.
        PickingSystem::PickInfo *pick = nullptr;
        if (picking)
        {
            if (picking->last_pick().valid)
            {
                pick = picking->mutable_last_pick();
            }
            else if (picking->hover_pick().valid)
            {
                pick = picking->mutable_hover_pick();
            }
        }

        if (!pick || pick->ownerName.empty())
        {
            ImGui::TextUnformatted("No selection for gizmo (pick or hover an instance).");
            return;
        }

        static ImGuizmo::OPERATION op = ImGuizmo::TRANSLATE;
        static ImGuizmo::MODE mode = ImGuizmo::LOCAL;

        ImGui::TextUnformatted("Operation");
        if (ImGui::RadioButton("Translate", op == ImGuizmo::TRANSLATE)) op = ImGuizmo::TRANSLATE;
        ImGui::SameLine();
        if (ImGui::RadioButton("Rotate", op == ImGuizmo::ROTATE)) op = ImGuizmo::ROTATE;
        ImGui::SameLine();
        if (ImGui::RadioButton("Scale", op == ImGuizmo::SCALE)) op = ImGuizmo::SCALE;

        ImGui::TextUnformatted("Mode");
        if (ImGui::RadioButton("Local", mode == ImGuizmo::LOCAL)) mode = ImGuizmo::LOCAL;
        ImGui::SameLine();
        if (ImGui::RadioButton("World", mode == ImGuizmo::WORLD)) mode = ImGuizmo::WORLD;

        // Resolve a dynamic instance transform for the current pick.
        glm::mat4 targetTransform(1.0f);
        enum class GizmoTarget
        {
            None,
            MeshInstance,
            GLTFInstance,
            Planet
        };
        GizmoTarget target = GizmoTarget::None;

        if (pick->ownerType == RenderObject::OwnerType::MeshInstance)
        {
            if (sceneMgr->getMeshInstanceTransformLocal(pick->ownerName, targetTransform))
            {
                target = GizmoTarget::MeshInstance;
                ImGui::Text("Editing mesh instance: %s", pick->ownerName.c_str());
            }
            else if (PlanetSystem *planets = sceneMgr->get_planet_system())
            {
                PlanetSystem::PlanetBody *body = planets->find_body_by_name(pick->ownerName);
                if (body)
                {
                    glm::vec3 tLocal = world_to_local(body->center_world, sceneMgr->get_world_origin());
                    targetTransform = make_trs_matrix(tLocal,
                                                      glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                                                      glm::vec3(1.0f));
                    target = GizmoTarget::Planet;
                    ImGui::Text("Editing planet: %s", pick->ownerName.c_str());
                    ImGui::Text("Radius: %.3f km", body->radius_m / 1000.0);
                    if (op == ImGuizmo::ROTATE)
                    {
                        ImGui::TextUnformatted("Note: planet rotation is not supported (use Translate/Scale).");
                    }
                }
            }
        }
        else if (pick->ownerType == RenderObject::OwnerType::GLTFInstance)
        {
            if (sceneMgr->getGLTFInstanceTransformLocal(pick->ownerName, targetTransform))
            {
                target = GizmoTarget::GLTFInstance;
                ImGui::Text("Editing glTF instance: %s", pick->ownerName.c_str());
            }
        }

        if (target == GizmoTarget::None)
        {
            ImGui::TextUnformatted("Gizmo only supports dynamic mesh/glTF instances.");
            return;
        }

        ImGuiIO &io = ImGui::GetIO();
        ImGuizmo::SetOrthographic(false);

        VkExtent2D swapExtent = eng->_swapchainManager
                                ? eng->_swapchainManager->swapchainExtent()
                                : VkExtent2D{1, 1};
        VkExtent2D drawExtent{
            static_cast<uint32_t>(static_cast<float>(eng->_logicalRenderExtent.width) * eng->renderScale),
            static_cast<uint32_t>(static_cast<float>(eng->_logicalRenderExtent.height) * eng->renderScale)
        };
        if (drawExtent.width == 0 || drawExtent.height == 0)
        {
            drawExtent = VkExtent2D{1, 1};
        }

        VkRect2D activeRect = vkutil::compute_letterbox_rect(drawExtent, swapExtent);
        const float fbScaleX = (io.DisplayFramebufferScale.x > 0.0f) ? io.DisplayFramebufferScale.x : 1.0f;
        const float fbScaleY = (io.DisplayFramebufferScale.y > 0.0f) ? io.DisplayFramebufferScale.y : 1.0f;
        const float rectX = static_cast<float>(activeRect.offset.x) / fbScaleX;
        const float rectY = static_cast<float>(activeRect.offset.y) / fbScaleY;
        const float rectW = static_cast<float>(activeRect.extent.width) / fbScaleX;
        const float rectH = static_cast<float>(activeRect.extent.height) / fbScaleY;

        ImGuizmo::SetDrawlist();
        ImGuizmo::SetRect(rectX, rectY, rectW, rectH);

        // Build a distance-based perspective projection for ImGuizmo instead of
        // using the engine's reversed-Z Vulkan projection.
        Camera &cam = sceneMgr->getMainCamera();
        float fovRad = glm::radians(cam.fovDegrees);
        float aspect = drawExtent.height > 0
                       ? static_cast<float>(drawExtent.width) / static_cast<float>(drawExtent.height)
                       : 1.0f;

        // Distance from camera to object; clamp to avoid degenerate planes.
        glm::vec3 camPos = sceneMgr->get_camera_local_position();
        glm::vec3 objPos = glm::vec3(targetTransform[3]);
        float dist = glm::length(objPos - camPos);
        if (!std::isfinite(dist) || dist <= 0.0f)
        {
            dist = 1.0f;
        }

        // Near/far based on distance: keep ratio reasonable for precision.
        float nearPlane = glm::max(0.05f, dist * 0.05f);
        float farPlane = glm::max(nearPlane * 50.0f, dist * 2.0f);

        glm::mat4 view = cam.getViewMatrix(sceneMgr->get_camera_local_position());
        glm::mat4 proj = glm::perspective(fovRad, aspect, nearPlane, farPlane);

        glm::mat4 before = targetTransform;

        ImDrawList* dl = ImGui::GetForegroundDrawList();
        ImGuizmo::SetDrawlist(dl);

        ImGuizmo::SetRect(rectX, rectY, rectW, rectH);
        ImGuizmo::Manipulate(&view[0][0], &proj[0][0],
                             op, mode,
                             &targetTransform[0][0]);

        bool changed = false;
        for (int c = 0; c < 4 && !changed; ++c)
        {
            for (int r = 0; r < 4; ++r)
            {
                if (before[c][r] != targetTransform[c][r])
                {
                    changed = true;
                    break;
                }
            }
        }

        if (changed)
        {
            switch (target)
            {
                case GizmoTarget::MeshInstance:
                    sceneMgr->setMeshInstanceTransformLocal(pick->ownerName, targetTransform);
                    break;
                case GizmoTarget::GLTFInstance:
                    sceneMgr->setGLTFInstanceTransformLocal(pick->ownerName, targetTransform);
                    break;
                case GizmoTarget::Planet:
                {
                    PlanetSystem *planets = sceneMgr->get_planet_system();
                    if (!planets)
                    {
                        break;
                    }

                    PlanetSystem::PlanetBody *body = planets->find_body_by_name(pick->ownerName);
                    if (!body)
                    {
                        break;
                    }

                    glm::vec3 tLocal{};
                    glm::quat r{};
                    glm::vec3 s{};
                    decompose_trs_matrix(targetTransform, tLocal, r, s);

                    if (op == ImGuizmo::TRANSLATE)
                    {
                        planets->set_planet_center(pick->ownerName, local_to_world(tLocal, sceneMgr->get_world_origin()));
                    }
                    else if (op == ImGuizmo::SCALE)
                    {
                        const double scale =
                            (static_cast<double>(std::abs(s.x)) +
                             static_cast<double>(std::abs(s.y)) +
                             static_cast<double>(std::abs(s.z))) / 3.0;
                        if (std::isfinite(scale) && scale > 0.0)
                        {
                            planets->set_planet_radius(pick->ownerName, body->radius_m * scale);
                        }
                    }
                    break;
                }
                default:
                    break;
            }

            // Keep pick debug info roughly in sync.
            pick->worldTransform = targetTransform;
            pick->worldPos = local_to_world(glm::vec3(targetTransform[3]), sceneMgr->get_world_origin());
        }
    }
} // namespace vk_engine::debug_ui
