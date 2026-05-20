#include "core/engine_ui/common.h"

namespace vk_engine::debug_ui
{
    static IBLPaths resolve_ibl_paths(VulkanEngine *eng, const IBLPaths &paths)
    {
        IBLPaths out = paths;
        if (!eng || !eng->_assetManager) return out;

        if (!out.specularCube.empty()) out.specularCube = eng->_assetManager->assetPath(out.specularCube);
        if (!out.diffuseCube.empty()) out.diffuseCube = eng->_assetManager->assetPath(out.diffuseCube);
        if (!out.brdfLut2D.empty()) out.brdfLut2D = eng->_assetManager->assetPath(out.brdfLut2D);
        if (!out.background2D.empty()) out.background2D = eng->_assetManager->assetPath(out.background2D);
        return out;
    }

    void ui_window(VulkanEngine *eng)
    {
        if (!eng || !eng->_window) return;

        int num_displays = SDL_GetNumVideoDisplays();
        if (num_displays <= 0)
        {
            ImGui::Text("No displays reported by SDL (%s)", SDL_GetError());
            return;
        }

        int current_display = SDL_GetWindowDisplayIndex(eng->_window);
        if (current_display < 0) current_display = eng->_windowDisplayIndex;
        current_display = std::clamp(current_display, 0, num_displays - 1);

        const char *cur_display_name = SDL_GetDisplayName(current_display);
        if (!cur_display_name) cur_display_name = "Unknown";

        ImGui::Text("Current: %s on display %d (%s)",
                    (eng->_windowMode == VulkanEngine::WindowMode::Windowed)
                        ? "Windowed"
                        : (eng->_windowMode == VulkanEngine::WindowMode::FullscreenDesktop)
                              ? "Borderless Fullscreen"
                              : "Exclusive Fullscreen",
                    current_display,
                    cur_display_name);

        static int pending_display = -1;
        static int pending_mode = -1; // 0 windowed, 1 borderless, 2 exclusive
        if (pending_display < 0) pending_display = current_display;
        if (pending_mode < 0) pending_mode = static_cast<int>(eng->_windowMode);

        ImGui::Separator();

        if (ImGui::BeginCombo("Monitor", cur_display_name))
        {
            for (int i = 0; i < num_displays; ++i)
            {
                const char *name = SDL_GetDisplayName(i);
                if (!name) name = "Unknown";
                const bool selected = (pending_display == i);
                if (ImGui::Selectable(name, selected))
                {
                    pending_display = i;
                }
                if (selected)
                {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        const char *mode_labels[] = {
            "Windowed",
            "Borderless (Fullscreen Desktop)",
            "Exclusive Fullscreen"
        };
        ImGui::Combo("Mode", &pending_mode, mode_labels, 3);

        ImGui::TextUnformatted("Apply triggers immediate swapchain recreation.");
        if (ImGui::Button("Apply"))
        {
            auto mode = static_cast<VulkanEngine::WindowMode>(std::clamp(pending_mode, 0, 2));
            eng->setWindowMode(mode, pending_display);

            // Re-sync pending selections with what SDL actually applied.
            pending_display = SDL_GetWindowDisplayIndex(eng->_window);
            if (pending_display < 0) pending_display = eng->_windowDisplayIndex;
            pending_display = std::clamp(pending_display, 0, num_displays - 1);
            pending_mode = static_cast<int>(eng->_windowMode);
        }
        ImGui::SameLine();
        if (ImGui::Button("Use Current"))
        {
            pending_display = current_display;
            pending_mode = static_cast<int>(eng->_windowMode);
        }

        ImGui::Separator();
        ImGui::TextUnformatted("HiDPI / Sizes");
        ImGui::Text("HiDPI enabled: %s", eng->_hiDpiEnabled ? "yes" : "no");

        int winW = 0, winH = 0;
        SDL_GetWindowSize(eng->_window, &winW, &winH);
        int drawW = 0, drawH = 0;
        SDL_Vulkan_GetDrawableSize(eng->_window, &drawW, &drawH);
        ImGui::Text("Window size: %d x %d", winW, winH);
        ImGui::Text("Drawable size: %d x %d", drawW, drawH);
        if (winW > 0 && winH > 0 && drawW > 0 && drawH > 0)
        {
            ImGui::Text("Drawable scale: %.3f x %.3f",
                        static_cast<float>(drawW) / static_cast<float>(winW),
                        static_cast<float>(drawH) / static_cast<float>(winH));
        }
        if (eng->_swapchainManager)
        {
            VkExtent2D sw = eng->_swapchainManager->swapchainExtent();
            ImGui::Text("Swapchain extent: %u x %u", sw.width, sw.height);
        }

        ImGui::Separator();
        ImGui::TextUnformatted("GPU");
        if (!eng->_deviceManager || !eng->_deviceManager->physicalDevice())
        {
            ImGui::TextUnformatted("No Vulkan device initialized.");
            return;
        }

        VkPhysicalDevice gpu = eng->_deviceManager->physicalDevice();
        VkPhysicalDeviceProperties props{};
        vkGetPhysicalDeviceProperties(gpu, &props);
        VkPhysicalDeviceMemoryProperties mem{};
        vkGetPhysicalDeviceMemoryProperties(gpu, &mem);

        auto type_str = [](VkPhysicalDeviceType t) -> const char*
        {
            switch (t)
            {
                case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU: return "Discrete";
                case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU: return "Integrated";
                case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU: return "Virtual";
                case VK_PHYSICAL_DEVICE_TYPE_CPU: return "CPU";
                default: return "Other";
            }
        };

        uint64_t device_local_bytes = 0;
        for (uint32_t i = 0; i < mem.memoryHeapCount; ++i)
        {
            if (mem.memoryHeaps[i].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT)
            {
                device_local_bytes += mem.memoryHeaps[i].size;
            }
        }
        const double vram_gib = static_cast<double>(device_local_bytes) / (1024.0 * 1024.0 * 1024.0);

        const uint32_t api = props.apiVersion;
        ImGui::Text("Name: %s", props.deviceName);
        ImGui::Text("Type: %s", type_str(props.deviceType));
        ImGui::Text("Vendor: 0x%04x  Device: 0x%04x", props.vendorID, props.deviceID);
        ImGui::Text("Vulkan API: %u.%u.%u", VK_VERSION_MAJOR(api), VK_VERSION_MINOR(api), VK_VERSION_PATCH(api));
        ImGui::Text("Driver: %u (0x%08x)", props.driverVersion, props.driverVersion);
        ImGui::Text("Device-local memory: %.2f GiB", vram_gib);
        ImGui::Text("RayQuery: %s  AccelStruct: %s",
                    eng->_deviceManager->supportsRayQuery() ? "yes" : "no",
                    eng->_deviceManager->supportsAccelerationStructure() ? "yes" : "no");
    }

    // Background / compute playground
    void ui_background(VulkanEngine *eng)
    {
        if (!eng || !eng->_renderPassManager) return;
        auto *background_pass = eng->_renderPassManager->getPass<BackgroundPass>();
        if (!background_pass)
        {
            ImGui::TextUnformatted("Background pass not available");
            return;
        }

        ComputeEffect &selected = background_pass->_backgroundEffects[background_pass->_currentEffect];

        ImGui::Text("Selected effect: %s", selected.name);
        ImGui::SliderInt("Effect Index", &background_pass->_currentEffect, 0,
                         (int) background_pass->_backgroundEffects.size() - 1);
        ImGui::InputFloat4("data1", reinterpret_cast<float *>(&selected.data.data1));
        ImGui::InputFloat4("data2", reinterpret_cast<float *>(&selected.data.data2));
        ImGui::InputFloat4("data3", reinterpret_cast<float *>(&selected.data.data3));
        ImGui::InputFloat4("data4", reinterpret_cast<float *>(&selected.data.data4));

        ImGui::Separator();
        ImGui::TextUnformatted("Render Resolution");

        static int pendingLogicalW = 0;
        static int pendingLogicalH = 0;
        if (pendingLogicalW <= 0 || pendingLogicalH <= 0)
        {
            pendingLogicalW = static_cast<int>(eng->_logicalRenderExtent.width);
            pendingLogicalH = static_cast<int>(eng->_logicalRenderExtent.height);
        }

        ImGui::InputInt("Logical Width", &pendingLogicalW);
        ImGui::InputInt("Logical Height", &pendingLogicalH);

        if (ImGui::Button("Apply Logical Resolution"))
        {
            uint32_t w = static_cast<uint32_t>(pendingLogicalW > 0 ? pendingLogicalW : 1);
            uint32_t h = static_cast<uint32_t>(pendingLogicalH > 0 ? pendingLogicalH : 1);
            eng->setLogicalRenderExtent(VkExtent2D{w, h});
        }
        ImGui::SameLine();
        if (ImGui::Button("720p"))
        {
            pendingLogicalW = 1280;
            pendingLogicalH = 720;
        }
        ImGui::SameLine();
        if (ImGui::Button("1080p"))
        {
            pendingLogicalW = 1920;
            pendingLogicalH = 1080;
        }
        ImGui::SameLine();
        if (ImGui::Button("1440p"))
        {
            pendingLogicalW = 2560;
            pendingLogicalH = 1440;
        }

        static float pendingScale = 1.0f;
        if (!ImGui::IsAnyItemActive())
        {
            pendingScale = eng->renderScale;
        }
        bool scaleChanged = ImGui::SliderFloat("Render Scale", &pendingScale, 0.25f, 2.0f);
        bool applyScale = scaleChanged && ImGui::IsItemDeactivatedAfterEdit();
        ImGui::SameLine();
        applyScale = ImGui::Button("Apply Scale") || applyScale;
        if (applyScale)
        {
            eng->setRenderScale(pendingScale);
        }
    }

    void ui_particles(VulkanEngine *eng)
    {
        if (!eng || !eng->_renderPassManager) return;
        auto *pass = eng->_renderPassManager->getPass<ParticlePass>();
        if (!pass)
        {
            ImGui::TextUnformatted("Particle pass not available");
            return;
        }

        const uint32_t freeCount = pass->free_particles();
        const uint32_t allocCount = pass->allocated_particles();
        ImGui::Text("Pool: %u allocated / %u free (max %u)", allocCount, freeCount, ParticlePass::k_max_particles);

        ImGui::Separator();

        static int newCount = 32768;
        newCount = std::max(newCount, 1);
        ImGui::InputInt("New System Particles", &newCount);
        ImGui::SameLine();
        if (ImGui::Button("Create"))
        {
            const uint32_t want = static_cast<uint32_t>(std::max(1, newCount));
            pass->create_system(want);
        }
        ImGui::SameLine();
        if (ImGui::Button("Create 32k"))
        {
            pass->create_system(32768);
        }
        ImGui::SameLine();
        if (ImGui::Button("Create 128k"))
        {
            pass->create_system(ParticlePass::k_max_particles);
        }

        ImGui::Separator();

        auto &systems = pass->systems();
        if (systems.empty())
        {
            ImGui::TextUnformatted("No particle systems. Create one above.");
            return;
        }

        static int selected = 0;
        selected = std::clamp(selected, 0, (int)systems.size() - 1);

        if (ImGui::BeginListBox("Systems"))
        {
            for (int i = 0; i < (int)systems.size(); ++i)
            {
                const auto &s = systems[i];
                char label[128];
                std::snprintf(label, sizeof(label), "#%u base=%u count=%u %s",
                              s.id, s.base, s.count, s.enabled ? "on" : "off");
                const bool isSelected = (selected == i);
                if (ImGui::Selectable(label, isSelected))
                {
                    selected = i;
                }
                if (isSelected) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndListBox();
        }

        selected = std::clamp(selected, 0, (int)systems.size() - 1);
        auto &s = systems[(size_t)selected];

        static std::vector<std::string> vfxKtx2;
        auto refresh_vfx_list = [&]() {
            vfxKtx2.clear();
            vfxKtx2.push_back(std::string{}); // None
            if (!eng || !eng->_assetManager) return;
            const auto &paths = eng->_assetManager->paths();
            if (paths.assets.empty()) return;
            std::error_code ec;
            std::filesystem::path vfxDir = paths.assets / "vfx";
            if (!std::filesystem::exists(vfxDir, ec) || ec) return;
            for (const auto &entry : std::filesystem::directory_iterator(vfxDir, ec))
            {
                if (ec) break;
                if (!entry.is_regular_file(ec) || ec) continue;
                const auto p = entry.path();
                if (p.extension() != ".ktx2" && p.extension() != ".KTX2") continue;
                vfxKtx2.push_back(std::string("vfx/") + p.filename().string());
            }
            std::sort(vfxKtx2.begin() + 1, vfxKtx2.end());
        };
        if (vfxKtx2.empty())
        {
            refresh_vfx_list();
        }

        ImGui::Separator();

        ImGui::Text("Selected: id=%u base=%u count=%u", s.id, s.base, s.count);
        ImGui::Checkbox("Enabled", &s.enabled);
        ImGui::SameLine();
        if (ImGui::Button("Reset (Respawn)"))
        {
            s.reset = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Destroy"))
        {
            const uint32_t id = s.id;
            pass->destroy_system(id);
            selected = 0;
            return;
        }

        const char *blendItems[] = {"Additive", "Alpha (block-sorted)"};
        int blend = (s.blend == ParticlePass::BlendMode::Alpha) ? 1 : 0;
        if (ImGui::Combo("Blend", &blend, blendItems, 2))
        {
            s.blend = (blend == 1) ? ParticlePass::BlendMode::Alpha : ParticlePass::BlendMode::Additive;
        }

        ImGui::Separator();

        static int pendingResizeCount = 0;
        if (!ImGui::IsAnyItemActive())
        {
            pendingResizeCount = (int)s.count;
        }
        ImGui::InputInt("Resize Count", &pendingResizeCount);
        ImGui::SameLine();
        if (ImGui::Button("Apply Resize"))
        {
            const uint32_t want = static_cast<uint32_t>(std::max(0, pendingResizeCount));
            pass->resize_system(s.id, want);
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Emitter");
        ImGui::InputFloat3("Position (local)", reinterpret_cast<float *>(&s.params.emitter_pos_local));
        ImGui::SliderFloat("Spawn Radius", &s.params.spawn_radius, 0.0f, 10.0f, "%.3f");
        ImGui::InputFloat3("Direction (local)", reinterpret_cast<float *>(&s.params.emitter_dir_local));
        ImGui::SliderFloat("Cone Angle (deg)", &s.params.cone_angle_degrees, 0.0f, 89.0f, "%.1f");

        ImGui::Separator();
        ImGui::TextUnformatted("Motion");
        ImGui::InputFloat("Min Speed", &s.params.min_speed);
        ImGui::InputFloat("Max Speed", &s.params.max_speed);
        ImGui::InputFloat("Min Life (s)", &s.params.min_life);
        ImGui::InputFloat("Max Life (s)", &s.params.max_life);
        ImGui::InputFloat("Min Size", &s.params.min_size);
        ImGui::InputFloat("Max Size", &s.params.max_size);
        ImGui::SliderFloat("Drag", &s.params.drag, 0.0f, 10.0f, "%.3f");
        ImGui::SliderFloat("Gravity (m/s^2)", &s.params.gravity, 0.0f, 30.0f, "%.2f");

        ImGui::Separator();
        ImGui::TextUnformatted("Rendering");
        ImGui::SliderFloat("Soft Depth (m)", &s.params.soft_depth_distance, 0.0f, 2.0f, "%.3f");

        if (ImGui::Button("Refresh VFX List"))
        {
            refresh_vfx_list();
        }
        ImGui::SameLine();
        if (ImGui::Button("Use Flame Defaults"))
        {
            s.flipbook_texture = "vfx/flame.ktx2";
            s.noise_texture = "vfx/simplex.ktx2";
            s.params.flipbook_cols = 8;
            s.params.flipbook_rows = 8;
            s.params.flipbook_fps = 30.0f;
            s.params.flipbook_intensity = 1.0f;
            s.params.noise_scale = 6.0f;
            s.params.noise_strength = 0.05f;
            s.params.noise_scroll = glm::vec2(0.0f, 0.0f);
            pass->preload_vfx_texture(s.flipbook_texture);
            pass->preload_vfx_texture(s.noise_texture);
        }

        auto combo_vfx = [&](const char *label, std::string &path) {
            const char *preview = path.empty() ? "None" : path.c_str();
            if (ImGui::BeginCombo(label, preview))
            {
                for (const auto &opt : vfxKtx2)
                {
                    const bool isNone = opt.empty();
                    const bool isSelected = (path == opt) || (path.empty() && isNone);
                    const char *name = isNone ? "None" : opt.c_str();
                    if (ImGui::Selectable(name, isSelected))
                    {
                        path = opt;
                        if (!path.empty())
                        {
                            pass->preload_vfx_texture(path);
                        }
                    }
                    if (isSelected) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
        };

        ImGui::Separator();
        ImGui::TextUnformatted("Flipbook");
        combo_vfx("Flipbook Texture", s.flipbook_texture);
        int cols = (int)s.params.flipbook_cols;
        int rows = (int)s.params.flipbook_rows;
        cols = std::max(cols, 1);
        rows = std::max(rows, 1);
        if (ImGui::InputInt("Flipbook Cols", &cols)) s.params.flipbook_cols = (uint32_t)std::max(cols, 1);
        if (ImGui::InputInt("Flipbook Rows", &rows)) s.params.flipbook_rows = (uint32_t)std::max(rows, 1);
        ImGui::SliderFloat("Flipbook FPS", &s.params.flipbook_fps, 0.0f, 120.0f, "%.1f");
        ImGui::SliderFloat("Flipbook Intensity", &s.params.flipbook_intensity, 0.0f, 8.0f, "%.3f");

        ImGui::Separator();
        ImGui::TextUnformatted("Noise");
        combo_vfx("Noise Texture", s.noise_texture);
        ImGui::SliderFloat("Noise Scale", &s.params.noise_scale, 0.0f, 32.0f, "%.3f");
        ImGui::SliderFloat("Noise Strength", &s.params.noise_strength, 0.0f, 1.0f, "%.3f");
        ImGui::InputFloat2("Noise Scroll", reinterpret_cast<float *>(&s.params.noise_scroll));

        ImGui::Separator();
        ImGui::TextUnformatted("Color");
        ImGui::ColorEdit4("Tint", reinterpret_cast<float *>(&s.params.color), ImGuiColorEditFlags_Float);
    }

    // IBL test grid spawner (spheres varying metallic/roughness)
    static void spawn_ibl_test(VulkanEngine *eng)
    {
        if (!eng || !eng->_assetManager || !eng->_sceneManager) return;
        using MC = GLTFMetallic_Roughness::MaterialConstants;

        std::vector<Vertex> verts;
        std::vector<uint32_t> inds;
        primitives::buildSphere(verts, inds, 24, 24);

        const float mVals[5] = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f};
        const float rVals[5] = {0.04f, 0.25f, 0.5f, 0.75f, 1.0f};
        const float spacing = 1.6f;
        const glm::vec3 origin(-spacing * 2.0f, 0.0f, -spacing * 2.0f);

        for (int iy = 0; iy < 5; ++iy)
        {
            for (int ix = 0; ix < 5; ++ix)
            {
                MC c{};
                c.colorFactors = glm::vec4(0.82f, 0.82f, 0.82f, 1.0f);
                c.metal_rough_factors = glm::vec4(mVals[ix], rVals[iy], 0.0f, 0.0f);
                const std::string base = fmt::format("ibltest.m{}_r{}", ix, iy);
                auto mat = eng->_assetManager->createMaterialFromConstants(base + ".mat", c, MaterialPass::MainColor);

                auto mesh = eng->_assetManager->createMesh(base + ".mesh",
                                                           std::span<Vertex>(verts.data(), verts.size()),
                                                           std::span<uint32_t>(inds.data(), inds.size()),
                                                           mat);

                const glm::vec3 pos = origin + glm::vec3(ix * spacing, 0.5f, iy * spacing);
                glm::mat4 M = glm::translate(glm::mat4(1.0f), pos);
                eng->_sceneManager->addMeshInstance(base + ".inst", mesh, M, BoundsType::Sphere);
                eng->_iblTestNames.push_back(base + ".inst");
                eng->_iblTestNames.push_back(base + ".mesh");
                eng->_iblTestNames.push_back(base + ".mat");
            }
        }

        // Chrome and glass extras
        {
            MC chrome{};
            chrome.colorFactors = glm::vec4(0.9f, 0.9f, 0.9f, 1.0f);
            chrome.metal_rough_factors = glm::vec4(1.0f, 0.06f, 0, 0);
            auto mat = eng->_assetManager->createMaterialFromConstants("ibltest.chrome.mat", chrome,
                                                                       MaterialPass::MainColor);
            auto mesh = eng->_assetManager->createMesh("ibltest.chrome.mesh",
                                                       std::span<Vertex>(verts.data(), verts.size()),
                                                       std::span<uint32_t>(inds.data(), inds.size()),
                                                       mat);
            glm::mat4 M = glm::translate(glm::mat4(1.0f), origin + glm::vec3(5.5f, 0.5f, 0.0f));
            eng->_sceneManager->addMeshInstance("ibltest.chrome.inst", mesh, M, BoundsType::Sphere);
            eng->_iblTestNames.insert(eng->_iblTestNames.end(),
                                      {"ibltest.chrome.inst", "ibltest.chrome.mesh", "ibltest.chrome.mat"});
        } {
            MC glass{};
            glass.colorFactors = glm::vec4(0.9f, 0.95f, 1.0f, 0.25f);
            glass.metal_rough_factors = glm::vec4(0.0f, 0.02f, 0, 0);
            auto mat = eng->_assetManager->createMaterialFromConstants("ibltest.glass.mat", glass,
                                                                       MaterialPass::Transparent);
            auto mesh = eng->_assetManager->createMesh("ibltest.glass.mesh",
                                                       std::span<Vertex>(verts.data(), verts.size()),
                                                       std::span<uint32_t>(inds.data(), inds.size()),
                                                       mat);
            glm::mat4 M = glm::translate(glm::mat4(1.0f), origin + glm::vec3(5.5f, 0.5f, 2.0f));
            eng->_sceneManager->addMeshInstance("ibltest.glass.inst", mesh, M, BoundsType::Sphere);
            eng->_iblTestNames.insert(eng->_iblTestNames.end(),
                                      {"ibltest.glass.inst", "ibltest.glass.mesh", "ibltest.glass.mat"});
        }
    }

    static void clear_ibl_test(VulkanEngine *eng)
    {
        if (!eng || !eng->_sceneManager || !eng->_assetManager) return;
        for (size_t i = 0; i < eng->_iblTestNames.size(); ++i)
        {
            const std::string &n = eng->_iblTestNames[i];
            // Remove instances and meshes by prefix
            if (n.ends_with(".inst")) eng->_sceneManager->removeMeshInstance(n);
            else if (n.ends_with(".mesh")) eng->_assetManager->removeMesh(n);
        }
        eng->_iblTestNames.clear();
    }

    void ui_ibl(VulkanEngine *eng)
    {
        if (!eng) return;

        if (ImGui::Button("Spawn IBL Test Grid")) { spawn_ibl_test(eng); }
        ImGui::SameLine();
        if (ImGui::Button("Clear IBL Test")) { clear_ibl_test(eng); }
        ImGui::TextUnformatted(
            "5x5 spheres: metallic across columns, roughness across rows.\nExtra: chrome + glass.");

        ImGui::Separator();

        // Post-processing: FXAA
        if (auto *fx = eng->_renderPassManager ? eng->_renderPassManager->getPass<FxaaPass>() : nullptr)
        {
            bool fxaaEnabled = fx->enabled();
            if (ImGui::Checkbox("FXAA", &fxaaEnabled))
            {
                fx->set_enabled(fxaaEnabled);
            }
            float edgeTh = fx->edge_threshold();
            if (ImGui::SliderFloat("FXAA Edge Threshold", &edgeTh, 0.01f, 0.5f))
            {
                fx->set_edge_threshold(edgeTh);
            }
            float edgeThMin = fx->edge_threshold_min();
            if (ImGui::SliderFloat("FXAA Edge Threshold Min", &edgeThMin, 0.0f, 0.1f))
            {
                fx->set_edge_threshold_min(edgeThMin);
            }
        }
        else
        {
            ImGui::TextUnformatted("FXAA pass not available");
        }
        ImGui::TextUnformatted("IBL Volumes (reflection probes)");

        if (!eng->_iblManager)
        {
            ImGui::TextUnformatted("IBLManager not available");
            return;
        }

        if (eng->_activeIBLVolume < 0)
        {
            ImGui::TextUnformatted("Active IBL: Global");
        }
        else
        {
            ImGui::Text("Active IBL: Volume %d", eng->_activeIBLVolume);
        }

        if (ImGui::Button("Add IBL Volume"))
        {
            VulkanEngine::IBLVolume vol{};
            if (eng->_sceneManager)
            {
                vol.center_world = eng->_sceneManager->getMainCamera().position_world;
            }
            vol.halfExtents = glm::vec3(10.0f, 10.0f, 10.0f);
            vol.paths = eng->_globalIBLPaths;
            eng->_iblVolumes.push_back(vol);
        }

        for (size_t i = 0; i < eng->_iblVolumes.size(); ++i)
        {
            auto &vol = eng->_iblVolumes[i];
            ImGui::PushID(static_cast<int>(i));
            ImGui::Separator();
            ImGui::Text("Volume %zu", i);
            ImGui::SameLine();
            if (ImGui::Button("Delete"))
            {
                const int idx = static_cast<int>(i);
                if (eng->_activeIBLVolume == idx)
                {
                    eng->_activeIBLVolume = -1;
                }
                else if (eng->_activeIBLVolume > idx)
                {
                    eng->_activeIBLVolume -= 1;
                }

                if (eng->_pendingIBLRequest.active)
                {
                    if (eng->_pendingIBLRequest.targetVolume == idx)
                    {
                        eng->_pendingIBLRequest.active = false;
                    }
                    else if (eng->_pendingIBLRequest.targetVolume > idx)
                    {
                        eng->_pendingIBLRequest.targetVolume -= 1;
                    }
                }

                eng->_iblVolumes.erase(eng->_iblVolumes.begin() + idx);
                ImGui::PopID();
                break;
            }
            ImGui::Checkbox("Enabled", &vol.enabled);
            {
                double c[3] = {vol.center_world.x, vol.center_world.y, vol.center_world.z};
                if (ImGui::InputScalarN("Center (world)", ImGuiDataType_Double, c, 3, nullptr, nullptr, "%.3f"))
                {
                    vol.center_world = WorldVec3(c[0], c[1], c[2]);
                }
            }
            {
                const char* shape_items[] = {"Box", "Sphere"};
                int shape_idx = (vol.shape == VulkanEngine::IBLVolumeShape::Sphere) ? 1 : 0;
                if (ImGui::Combo("Shape", &shape_idx, shape_items, IM_ARRAYSIZE(shape_items)))
                {
                    VulkanEngine::IBLVolumeShape new_shape =
                        (shape_idx == 1) ? VulkanEngine::IBLVolumeShape::Sphere : VulkanEngine::IBLVolumeShape::Box;
                    if (new_shape != vol.shape)
                    {
                        if (new_shape == VulkanEngine::IBLVolumeShape::Sphere)
                        {
                            vol.radius = std::max({vol.halfExtents.x, vol.halfExtents.y, vol.halfExtents.z});
                        }
                        else
                        {
                            vol.halfExtents = glm::vec3(vol.radius);
                        }
                        vol.shape = new_shape;
                    }
                }
            }
            if (vol.shape == VulkanEngine::IBLVolumeShape::Sphere)
            {
                ImGui::InputFloat("Radius", &vol.radius);
                vol.radius = std::max(vol.radius, 0.0f);
            }
            else
            {
                ImGui::InputFloat3("Half Extents", &vol.halfExtents.x);
            }

            // Simple path editors; store absolute or engine-local paths.
            char specBuf[256]{};
            char diffBuf[256]{};
            char bgBuf[256]{};
            char brdfBuf[256]{};
            std::strncpy(specBuf, vol.paths.specularCube.c_str(), sizeof(specBuf) - 1);
            std::strncpy(diffBuf, vol.paths.diffuseCube.c_str(), sizeof(diffBuf) - 1);
            std::strncpy(bgBuf, vol.paths.background2D.c_str(), sizeof(bgBuf) - 1);
            std::strncpy(brdfBuf, vol.paths.brdfLut2D.c_str(), sizeof(brdfBuf) - 1);

            if (ImGui::InputText("Specular path", specBuf, IM_ARRAYSIZE(specBuf)))
            {
                vol.paths.specularCube = specBuf;
            }
            if (ImGui::InputText("Diffuse path", diffBuf, IM_ARRAYSIZE(diffBuf)))
            {
                vol.paths.diffuseCube = diffBuf;
            }
            if (ImGui::InputText("Background path", bgBuf, IM_ARRAYSIZE(bgBuf)))
            {
                vol.paths.background2D = bgBuf;
            }
            if (ImGui::InputText("BRDF LUT path", brdfBuf, IM_ARRAYSIZE(brdfBuf)))
            {
                vol.paths.brdfLut2D = brdfBuf;
            }

            if (ImGui::Button("Reload This Volume IBL"))
            {
                if (eng->_iblManager && vol.enabled)
                {
                    vol.paths = resolve_ibl_paths(eng, vol.paths);
                    if (eng->_iblManager->loadAsync(vol.paths))
                    {
                        eng->_pendingIBLRequest.active = true;
                        eng->_pendingIBLRequest.targetVolume = static_cast<int>(i);
                        eng->_pendingIBLRequest.paths = vol.paths;
                    }
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Set As Global IBL"))
            {
                vol.paths = resolve_ibl_paths(eng, vol.paths);
                eng->_globalIBLPaths = vol.paths;
                if (eng->_iblManager)
                {
                    if (eng->_iblManager->loadAsync(eng->_globalIBLPaths))
                    {
                        eng->_pendingIBLRequest.active = true;
                        eng->_pendingIBLRequest.targetVolume = -1;
                        eng->_pendingIBLRequest.paths = eng->_globalIBLPaths;
                        eng->_hasGlobalIBL = false;
                    }
                }
            }

            ImGui::PopID();
        }
    }

} // namespace vk_engine::debug_ui
