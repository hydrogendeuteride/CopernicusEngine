#pragma once
#include "vk_renderpass.h"
#include "core/vk_types.h"
#include <render/rg_types.h>

class ImGuiPass : public IRenderPass
{
public:
    void init(EngineContext *context) override;

    void cleanup() override;

    void execute(VkCommandBuffer cmd) override;

    const char *getName() const override { return "ImGui"; }

    void register_graph(class RenderGraph *graph,
                        RGImageHandle swapchainHandle);

private:
    EngineContext *_context = nullptr;

    void draw_imgui(VkCommandBuffer cmd,
                    EngineContext *context,
                    const class RGPassResources &resources,
                    RGImageHandle targetHandle) const;

    DeletionQueue _deletionQueue;
};
