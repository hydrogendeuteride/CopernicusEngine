#include "core/engine_ui/common.h"

namespace vk_engine::debug_ui
{
    static const char *stateName(uint8_t s)
    {
        switch (s)
        {
            case 0: return "Unloaded";
            case 1: return "Loading";
            case 2: return "Resident";
            case 3: return "Evicted";
            default: return "?";
        }
    }

    void ui_textures(VulkanEngine *eng)
    {
        if (!eng || !eng->_textureCache)
        {
            ImGui::TextUnformatted("TextureCache not available");
            return;
        }
        DeviceManager *dev = eng->_deviceManager.get();
        VmaAllocator alloc = dev ? dev->allocator() : VK_NULL_HANDLE;
        unsigned long long devLocalBudget = 0, devLocalUsage = 0;
        if (alloc)
        {
            const VkPhysicalDeviceMemoryProperties *memProps = nullptr;
            vmaGetMemoryProperties(alloc, &memProps);
            VmaBudget budgets[VK_MAX_MEMORY_HEAPS] = {};
            vmaGetHeapBudgets(alloc, budgets);
            if (memProps)
            {
                for (uint32_t i = 0; i < memProps->memoryHeapCount; ++i)
                {
                    if (memProps->memoryHeaps[i].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT)
                    {
                        devLocalBudget += budgets[i].budget;
                        devLocalUsage += budgets[i].usage;
                    }
                }
            }
        }

        const size_t texBudget = eng->queryTextureBudgetBytes();
        eng->_textureCache->setGpuBudgetBytes(texBudget);
        const size_t resBytes = eng->_textureCache->residentBytes();
        const size_t cpuSrcBytes = eng->_textureCache->cpuSourceBytes();
        const size_t uploadBytesLastPump = eng->_textureCache->uploaded_bytes_last_pump();
        const size_t decodeQueueDepth = eng->_textureCache->decode_queue_depth();
        const size_t readyQueueDepth = eng->_textureCache->ready_queue_depth();
        ImGui::Text("Device local: %.1f / %.1f MiB",
                    (double) devLocalUsage / 1048576.0,
                    (double) devLocalBudget / 1048576.0);
        ImGui::Text("Texture budget: %.1f MiB", (double) texBudget / 1048576.0);
        ImGui::Text("Resident textures: %.1f MiB", (double) resBytes / 1048576.0);
        ImGui::Text("CPU source bytes: %.1f MiB", (double) cpuSrcBytes / 1048576.0);
        ImGui::Text("Uploaded last frame: %.2f MiB", (double) uploadBytesLastPump / 1048576.0);
        ImGui::Text("Decode queue: %zu  Ready queue: %zu", decodeQueueDepth, readyQueueDepth);
        ImGui::SameLine();
        if (ImGui::Button("Trim To Budget Now"))
        {
            eng->_textureCache->evictToBudget(texBudget);
        }

        // Controls
        static int loadsPerPump = 4;
        loadsPerPump = eng->_textureCache->maxLoadsPerPump();
        if (ImGui::SliderInt("Loads/Frame", &loadsPerPump, 1, 16))
        {
            eng->_textureCache->setMaxLoadsPerPump(loadsPerPump);
        }
        static int uploadBudgetMiB = 128;
        uploadBudgetMiB = (int) (eng->_textureCache->maxBytesPerPump() / 1048576ull);
        if (ImGui::SliderInt("Upload Budget (MiB)", &uploadBudgetMiB, 16, 2048))
        {
            eng->_textureCache->setMaxBytesPerPump((size_t) uploadBudgetMiB * 1048576ull);
        }
        static bool keepSources = false;
        keepSources = eng->_textureCache->keepSourceBytes();
        if (ImGui::Checkbox("Keep Source Bytes", &keepSources))
        {
            eng->_textureCache->setKeepSourceBytes(keepSources);
        }
        static int cpuBudgetMiB = 64;
        cpuBudgetMiB = (int) (eng->_textureCache->cpuSourceBudget() / 1048576ull);
        if (ImGui::SliderInt("CPU Source Budget (MiB)", &cpuBudgetMiB, 0, 2048))
        {
            eng->_textureCache->setCpuSourceBudget((size_t) cpuBudgetMiB * 1048576ull);
        }
        static int maxUploadDim = 4096;
        maxUploadDim = (int) eng->_textureCache->maxUploadDimension();
        if (ImGui::SliderInt("Max Upload Dimension", &maxUploadDim, 0, 8192))
        {
            eng->_textureCache->setMaxUploadDimension((uint32_t) std::max(0, maxUploadDim));
        }

        TextureCache::DebugStats stats{};
        std::vector<TextureCache::DebugRow> rows;
        eng->_textureCache->debugSnapshot(rows, stats);
        ImGui::Text("Counts  R:%zu  U:%zu  E:%zu",
                    stats.countResident,
                    stats.countUnloaded,
                    stats.countEvicted);

        const int topN = 12;
        if (ImGui::BeginTable("texrows", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
        {
            ImGui::TableSetupColumn("MiB", ImGuiTableColumnFlags_WidthFixed, 80);
            ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthFixed, 90);
            ImGui::TableSetupColumn("LastUsed", ImGuiTableColumnFlags_WidthFixed, 90);
            ImGui::TableSetupColumn("Name");
            ImGui::TableHeadersRow();
            int count = 0;
            for (const auto &r: rows)
            {
                if (count++ >= topN) break;
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("%.2f", (double) r.bytes / 1048576.0);
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(stateName(r.state));
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%u", r.lastUsed);
                ImGui::TableSetColumnIndex(3);
                ImGui::TextUnformatted(r.name.c_str());
            }
            ImGui::EndTable();
        }
    }

    static const char *job_state_name(AsyncAssetLoader::JobState s)
    {
        using JS = AsyncAssetLoader::JobState;
        switch (s)
        {
            case JS::Pending:   return "Pending";
            case JS::Running:   return "Running";
            case JS::Completed: return "Completed";
            case JS::Failed:    return "Failed";
            case JS::Cancelled: return "Cancelled";
            default:            return "?";
        }
    }

    void ui_async_assets(VulkanEngine *eng)
    {
        if (!eng || !eng->_asyncLoader)
        {
            ImGui::TextUnformatted("AsyncAssetLoader not available");
            return;
        }

        std::vector<AsyncAssetLoader::DebugJob> jobs;
        eng->_asyncLoader->debugSnapshot(jobs);

        ImGui::Text("Active jobs: %zu", jobs.size());
        ImGui::Separator();

        if (!jobs.empty())
        {
            if (ImGui::BeginTable("async_jobs", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
            {
                ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthFixed, 40);
                ImGui::TableSetupColumn("Scene");
                ImGui::TableSetupColumn("Model");
                ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthFixed, 90);
                ImGui::TableSetupColumn("Progress", ImGuiTableColumnFlags_WidthFixed, 180);
                ImGui::TableHeadersRow();

                for (const auto &j : jobs)
                {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%u", j.id);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextUnformatted(j.scene_name.c_str());
                    ImGui::TableSetColumnIndex(2);
                    ImGui::TextUnformatted(j.model_relative_path.c_str());
                    ImGui::TableSetColumnIndex(3);
                    ImGui::TextUnformatted(job_state_name(j.state));
                    ImGui::TableSetColumnIndex(4);
                    float p = j.progress;
                    ImGui::ProgressBar(p, ImVec2(-FLT_MIN, 0.0f));
                    if (j.texture_count > 0)
                    {
                        ImGui::SameLine();
                        ImGui::Text("(%zu/%zu tex)", j.textures_resident, j.texture_count);
                    }
                }
                ImGui::EndTable();
            }
        }
        else
        {
            ImGui::TextUnformatted("No async asset jobs currently running.");
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Spawn async glTF instance");
        static char gltfPath[256] = "mirage_c/scene.gltf";
        static char gltfName[128] = "async_gltf_01";
        static float gltfPos[3] = {0.0f, 0.0f, 0.0f};
        static float gltfRot[3] = {0.0f, 0.0f, 0.0f};
        static float gltfScale[3] = {1.0f, 1.0f, 1.0f};
        ImGui::InputText("Model path (assets/models/...)", gltfPath, IM_ARRAYSIZE(gltfPath));
        ImGui::InputText("Instance name", gltfName, IM_ARRAYSIZE(gltfName));
        ImGui::InputFloat3("Position", gltfPos);
        ImGui::InputFloat3("Rotation (deg XYZ)", gltfRot);
        ImGui::InputFloat3("Scale", gltfScale);
        if (ImGui::Button("Load glTF async"))
        {
            glm::mat4 T = glm::translate(glm::mat4(1.0f), glm::vec3(gltfPos[0], gltfPos[1], gltfPos[2]));
            glm::mat4 R = glm::eulerAngleXYZ(glm::radians(gltfRot[0]),
                                             glm::radians(gltfRot[1]),
                                             glm::radians(gltfRot[2]));
            glm::mat4 S = glm::scale(glm::mat4(1.0f), glm::vec3(gltfScale[0], gltfScale[1], gltfScale[2]));
            glm::mat4 M = T * R * S;
            eng->loadGLTFAsync(gltfName, gltfPath, M);
            eng->preloadInstanceTextures(gltfName);
        }
    }

} // namespace vk_engine::debug_ui
