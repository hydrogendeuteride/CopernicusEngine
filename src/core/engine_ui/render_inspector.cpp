#include "core/engine_ui/common.h"

namespace vk_engine::debug_ui
{
    void ui_shadows(VulkanEngine *eng)
    {
        if (!eng) return;
        const bool rq = eng->_deviceManager->supportsRayQuery();
        const bool as = eng->_deviceManager->supportsAccelerationStructure();
        ImGui::Text("RayQuery: %s", rq ? "supported" : "not available");
        ImGui::Text("AccelStruct: %s", as ? "supported" : "not available");
        ImGui::Separator();

        auto &ss = eng->_context->shadowSettings;

        // Global on/off toggle for all shadowing.
        ImGui::Checkbox("Enable Shadows", &ss.enabled);
        ImGui::Separator();

        ImGui::BeginDisabled(!ss.enabled);
        int mode = static_cast<int>(ss.mode);
        ImGui::TextUnformatted("Shadow Mode");
        ImGui::RadioButton("Clipmap only", &mode, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Clipmap + RT", &mode, 1);
        ImGui::SameLine();
        ImGui::RadioButton("RT only", &mode, 2);
        if (!(rq && as) && mode != 0) mode = 0; // guard for unsupported HW
        ss.mode = static_cast<uint32_t>(mode);
        ss.hybridRayQueryEnabled = ss.enabled && (ss.mode != 0);

        ImGui::Separator();
        ImGui::SliderFloat("Shadow min visibility", &ss.shadowMinVisibility, 0.0f, 0.5f, "%.2f");
        ImGui::TextWrapped("Keeps some direct sun light in shadow (0 = full dark).");

        ImGui::Separator();
        ImGui::TextUnformatted("Shadow Map Resolution");
        {
            static constexpr uint32_t kResValues[] = {256u, 512u, 1024u, 2048u, 4096u, 8192u};
            static const char *kResLabels[] = {"256", "512", "1024", "2048", "4096", "8192"};

            const uint32_t effective = eng->_context ? eng->_context->getShadowMapResolution() : ss.shadowMapResolution;
            int currentIdx = -1;
            for (int i = 0; i < static_cast<int>(IM_ARRAYSIZE(kResValues)); ++i)
            {
                if (kResValues[i] == effective)
                {
                    currentIdx = i;
                    break;
                }
            }

            int idx = (currentIdx >= 0) ? currentIdx : 3; // default to 2048 when custom
            if (ImGui::Combo("Preset", &idx, kResLabels, IM_ARRAYSIZE(kResLabels)))
            {
                if (eng->_context) eng->_context->setShadowMapResolution(kResValues[idx]);
                else ss.shadowMapResolution = kResValues[idx];
            }

            int custom = static_cast<int>(ss.shadowMapResolution);
            if (ImGui::InputInt("Custom", &custom, 256, 1024, ImGuiInputTextFlags_EnterReturnsTrue))
            {
                if (eng->_context) eng->_context->setShadowMapResolution(static_cast<uint32_t>(custom));
                else ss.shadowMapResolution = static_cast<uint32_t>(custom);
            }
            ImGui::Text("Effective: %u", effective);
        }

        ImGui::BeginDisabled(ss.mode != 1u);
        ImGui::TextUnformatted("Cascades using ray assist:");
        for (int i = 0; i < 4; ++i)
        {
            bool on = (ss.hybridRayCascadesMask >> i) & 1u;
            std::string label = std::string("C") + std::to_string(i);
            if (ImGui::Checkbox(label.c_str(), &on))
            {
                if (on) ss.hybridRayCascadesMask |= (1u << i);
                else ss.hybridRayCascadesMask &= ~(1u << i);
            }
            if (i != 3) ImGui::SameLine();
        }
        ImGui::SliderFloat("N·L threshold", &ss.hybridRayNoLThreshold, 0.0f, 1.0f, "%.2f");
        ImGui::EndDisabled();

        ImGui::Separator();
        ImGui::TextUnformatted("Punctual (Point/Spot) Shadows");
        int punctualMode = static_cast<int>(ss.punctualMode);
        ImGui::RadioButton("Off##punctual_mode", &punctualMode, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Shadow map##punctual_mode", &punctualMode, 1);
        ImGui::SameLine();
        ImGui::RadioButton("RT only##punctual_mode", &punctualMode, 2);
        ImGui::SameLine();
        ImGui::RadioButton("Hybrid##punctual_mode", &punctualMode, 3);
        if (!(rq && as) && (punctualMode == 2 || punctualMode == 3))
        {
            punctualMode = 1;
        }
        ss.punctualMode = static_cast<uint32_t>(punctualMode);

        auto clamp_u32 = [](uint32_t v, uint32_t lo, uint32_t hi) -> uint32_t
        {
            return std::min(std::max(v, lo), hi);
        };

        ss.maxShadowedSpotLights = clamp_u32(ss.maxShadowedSpotLights, 0u, kMaxShadowedSpotLights);
        ss.maxShadowedPointLights = clamp_u32(ss.maxShadowedPointLights, 0u, kMaxShadowedPointLights);
        ss.spotShadowMapResolution = clamp_u32(ss.spotShadowMapResolution, 128u, 8192u);
        ss.pointShadowMapResolution = clamp_u32(ss.pointShadowMapResolution, 128u, 8192u);
        ss.hybridRtMaxSpotLights = clamp_u32(ss.hybridRtMaxSpotLights, 0u, kMaxShadowedSpotLights);
        ss.hybridRtMaxPointLights = clamp_u32(ss.hybridRtMaxPointLights, 0u, kMaxShadowedPointLights);
        ss.punctualHybridRayNoLThreshold = glm::clamp(ss.punctualHybridRayNoLThreshold, 0.0f, 1.0f);

        int maxSpot = static_cast<int>(ss.maxShadowedSpotLights);
        int maxPoint = static_cast<int>(ss.maxShadowedPointLights);
        ImGui::SliderInt("Max shadowed spot", &maxSpot, 0, static_cast<int>(kMaxShadowedSpotLights));
        ImGui::SliderInt("Max shadowed point", &maxPoint, 0, static_cast<int>(kMaxShadowedPointLights));
        ss.maxShadowedSpotLights = static_cast<uint32_t>(maxSpot);
        ss.maxShadowedPointLights = static_cast<uint32_t>(maxPoint);

        int spotRes = static_cast<int>(ss.spotShadowMapResolution);
        int pointRes = static_cast<int>(ss.pointShadowMapResolution);
        ImGui::InputInt("Spot shadow resolution", &spotRes, 64, 256);
        ImGui::InputInt("Point shadow resolution", &pointRes, 64, 256);
        ss.spotShadowMapResolution = clamp_u32(static_cast<uint32_t>(std::max(spotRes, 1)), 128u, 8192u);
        ss.pointShadowMapResolution = clamp_u32(static_cast<uint32_t>(std::max(pointRes, 1)), 128u, 8192u);

        ImGui::BeginDisabled(ss.punctualMode != 3u && ss.punctualMode != 2u);
        int rtSpot = static_cast<int>(ss.hybridRtMaxSpotLights);
        int rtPoint = static_cast<int>(ss.hybridRtMaxPointLights);
        ImGui::SliderInt("RT spot budget", &rtSpot, 0, static_cast<int>(kMaxShadowedSpotLights));
        ImGui::SliderInt("RT point budget", &rtPoint, 0, static_cast<int>(kMaxShadowedPointLights));
        ss.hybridRtMaxSpotLights = static_cast<uint32_t>(rtSpot);
        ss.hybridRtMaxPointLights = static_cast<uint32_t>(rtPoint);
        ImGui::SliderFloat("Punctual hybrid N·L", &ss.punctualHybridRayNoLThreshold, 0.0f, 1.0f, "%.2f");
        ImGui::EndDisabled();
        ImGui::SliderFloat("Spot depth bias", &ss.spotShadowDepthBias, 0.0f, 0.02f, "%.5f");
        ImGui::SliderFloat("Point depth bias", &ss.pointShadowDepthBias, 0.0f, 0.02f, "%.5f");
        ImGui::EndDisabled();

        ImGui::Separator();
        ImGui::TextUnformatted("Analytic Planet Shadow");
        ImGui::SliderFloat("Sun angular radius (deg)", &ss.planetSunAngularRadiusDeg, 0.0f, 2.0f, "%.3f");
        ImGui::TextWrapped("Soft penumbra for planet->scene directional shadows. Set to 0 for a hard edge.");

        ImGui::Separator();
        ImGui::TextWrapped(
            "Clipmap only: raster PCF+RPDB. Clipmap+RT: PCF assisted by ray query at low N·L. RT only: skip shadow maps and use ray tests only.");
    }

    // Render Graph inspection (passes, images, buffers)
    void ui_render_graph(VulkanEngine *eng)
    {
        if (!eng || !eng->_renderGraph)
        {
            ImGui::TextUnformatted("RenderGraph not available");
            return;
        }
        auto &graph = *eng->_renderGraph;

        std::vector<RenderGraph::RGDebugPassInfo> passInfos;
        graph.debug_get_passes(passInfos);
        if (ImGui::Button("Reload Pipelines")) { eng->_pipelineManager->hotReloadChanged(); }
        ImGui::SameLine();
        ImGui::Text("%zu passes", passInfos.size());

        auto draw_use_summary = [](const std::vector<std::string> &uses)
        {
            if (uses.empty())
            {
                ImGui::TextUnformatted("-");
                return;
            }
            const size_t previewCount = std::min<size_t>(2, uses.size());
            std::string summary;
            for (size_t i = 0; i < previewCount; ++i)
            {
                if (i > 0) summary += ", ";
                summary += uses[i];
            }
            if (uses.size() > previewCount)
            {
                summary += ", +";
                summary += std::to_string(uses.size() - previewCount);
            }
            ImGui::TextUnformatted(summary.c_str());
            if (ImGui::IsItemHovered() && uses.size() > previewCount)
            {
                ImGui::BeginTooltip();
                for (const auto &u : uses) ImGui::TextUnformatted(u.c_str());
                ImGui::EndTooltip();
            }
        };

        if (ImGui::BeginTable("passes", 8, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
        {
            ImGui::TableSetupColumn("Enable", ImGuiTableColumnFlags_WidthFixed, 70);
            ImGui::TableSetupColumn("Name");
            ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 80);
            ImGui::TableSetupColumn("GPU ms", ImGuiTableColumnFlags_WidthFixed, 70);
            ImGui::TableSetupColumn("CPU rec ms", ImGuiTableColumnFlags_WidthFixed, 90);
            ImGui::TableSetupColumn("Imgs", ImGuiTableColumnFlags_WidthFixed, 55);
            ImGui::TableSetupColumn("Bufs", ImGuiTableColumnFlags_WidthFixed, 55);
            ImGui::TableSetupColumn("Attachments", ImGuiTableColumnFlags_WidthFixed, 100);
            ImGui::TableHeadersRow();

            auto typeName = [](RGPassType t) {
                switch (t)
                {
                    case RGPassType::Graphics: return "Graphics";
                    case RGPassType::Compute: return "Compute";
                    case RGPassType::Transfer: return "Transfer";
                    default: return "?";
                }
            };

            for (size_t i = 0; i < passInfos.size(); ++i)
            {
                auto &pi = passInfos[i];
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                bool enabled = true;
                if (auto it = eng->_rgPassToggles.find(pi.name); it != eng->_rgPassToggles.end()) enabled = it->second;
                std::string chkId = std::string("##en") + std::to_string(i);
                if (ImGui::Checkbox(chkId.c_str(), &enabled))
                {
                    eng->_rgPassToggles[pi.name] = enabled;
                }
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(pi.name.c_str());
                ImGui::TableSetColumnIndex(2);
                ImGui::TextUnformatted(typeName(pi.type));
                ImGui::TableSetColumnIndex(3);
                if (pi.gpuMillis >= 0.0f) ImGui::Text("%.2f", pi.gpuMillis);
                else ImGui::TextUnformatted("-");
                ImGui::TableSetColumnIndex(4);
                if (pi.cpuMillis >= 0.0f) ImGui::Text("%.2f", pi.cpuMillis);
                else ImGui::TextUnformatted("-");
                ImGui::TableSetColumnIndex(5);
                ImGui::Text("%u/%u", pi.imageReads, pi.imageWrites);
                ImGui::TableSetColumnIndex(6);
                ImGui::Text("%u/%u", pi.bufferReads, pi.bufferWrites);
                ImGui::TableSetColumnIndex(7);
                ImGui::Text("%u%s", pi.colorAttachmentCount, pi.hasDepth ? "+D" : "");
            }
            ImGui::EndTable();
        }

        if (ImGui::CollapsingHeader("Images", ImGuiTreeNodeFlags_DefaultOpen))
        {
            std::vector<RenderGraph::RGDebugImageInfo> imgs;
            graph.debug_get_images(imgs);
            if (ImGui::BeginTable("images", 8, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
            {
                ImGui::TableSetupColumn("Id", ImGuiTableColumnFlags_WidthFixed, 40);
                ImGui::TableSetupColumn("Name");
                ImGui::TableSetupColumn("Fmt", ImGuiTableColumnFlags_WidthFixed, 120);
                ImGui::TableSetupColumn("Extent", ImGuiTableColumnFlags_WidthFixed, 120);
                ImGui::TableSetupColumn("Imported", ImGuiTableColumnFlags_WidthFixed, 70);
                ImGui::TableSetupColumn("Life", ImGuiTableColumnFlags_WidthFixed, 220);
                ImGui::TableSetupColumn("Readers");
                ImGui::TableSetupColumn("Writers");
                ImGui::TableHeadersRow();
                for (const auto &im: imgs)
                {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%u", im.id);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextUnformatted(im.name.c_str());
                    ImGui::TableSetColumnIndex(2);
                    ImGui::TextUnformatted(string_VkFormat(im.format));
                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("%ux%u", im.extent.width, im.extent.height);
                    ImGui::TableSetColumnIndex(4);
                    ImGui::TextUnformatted(im.imported ? "yes" : "no");
                    ImGui::TableSetColumnIndex(5);
                    ImGui::Text("%d..%d  (%s -> %s)",
                                im.firstUse,
                                im.lastUse,
                                im.firstUsePass.empty() ? "-" : im.firstUsePass.c_str(),
                                im.lastUsePass.empty() ? "-" : im.lastUsePass.c_str());
                    ImGui::TableSetColumnIndex(6);
                    draw_use_summary(im.readers);
                    ImGui::TableSetColumnIndex(7);
                    draw_use_summary(im.writers);
                }
                ImGui::EndTable();
            }
        }

        if (ImGui::CollapsingHeader("Buffers"))
        {
            std::vector<RenderGraph::RGDebugBufferInfo> bufs;
            graph.debug_get_buffers(bufs);
            if (ImGui::BeginTable("buffers", 7, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
            {
                ImGui::TableSetupColumn("Id", ImGuiTableColumnFlags_WidthFixed, 40);
                ImGui::TableSetupColumn("Name");
                ImGui::TableSetupColumn("Size", ImGuiTableColumnFlags_WidthFixed, 100);
                ImGui::TableSetupColumn("Imported", ImGuiTableColumnFlags_WidthFixed, 70);
                ImGui::TableSetupColumn("Life", ImGuiTableColumnFlags_WidthFixed, 220);
                ImGui::TableSetupColumn("Readers");
                ImGui::TableSetupColumn("Writers");
                ImGui::TableHeadersRow();
                for (const auto &bf: bufs)
                {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%u", bf.id);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextUnformatted(bf.name.c_str());
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%zu", (size_t) bf.size);
                    ImGui::TableSetColumnIndex(3);
                    ImGui::TextUnformatted(bf.imported ? "yes" : "no");
                    ImGui::TableSetColumnIndex(4);
                    ImGui::Text("%d..%d  (%s -> %s)",
                                bf.firstUse,
                                bf.lastUse,
                                bf.firstUsePass.empty() ? "-" : bf.firstUsePass.c_str(),
                                bf.lastUsePass.empty() ? "-" : bf.lastUsePass.c_str());
                    ImGui::TableSetColumnIndex(5);
                    draw_use_summary(bf.readers);
                    ImGui::TableSetColumnIndex(6);
                    draw_use_summary(bf.writers);
                }
                ImGui::EndTable();
            }
        }
    }

    // Pipeline manager (graphics)
    void ui_pipelines(VulkanEngine *eng)
    {
        if (!eng || !eng->_pipelineManager)
        {
            ImGui::TextUnformatted("PipelineManager not available");
            return;
        }
        std::vector<PipelineManager::GraphicsPipelineDebugInfo> pipes;
        eng->_pipelineManager->debugGetGraphics(pipes);
        if (ImGui::Button("Reload Changed")) { eng->_pipelineManager->hotReloadChanged(); }
        ImGui::SameLine();
        ImGui::Text("%zu graphics pipelines", pipes.size());
        if (ImGui::BeginTable("gfxpipes", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
        {
            ImGui::TableSetupColumn("Name");
            ImGui::TableSetupColumn("VS");
            ImGui::TableSetupColumn("FS");
            ImGui::TableSetupColumn("Valid", ImGuiTableColumnFlags_WidthFixed, 60);
            ImGui::TableHeadersRow();
            for (const auto &p: pipes)
            {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(p.name.c_str());
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(p.vertexShaderPath.c_str());
                ImGui::TableSetColumnIndex(2);
                ImGui::TextUnformatted(p.fragmentShaderPath.c_str());
                ImGui::TableSetColumnIndex(3);
                ImGui::TextUnformatted(p.valid ? "yes" : "no");
            }
            ImGui::EndTable();
        }
    }

} // namespace vk_engine::debug_ui
