#pragma once

#include "vk_renderpass.h"
#include "render/rg_types.h"

class TransparentPass : public IRenderPass
{
public:
    void init(EngineContext *context) override;
    void execute(VkCommandBuffer cmd) override;
    void cleanup() override;
    const char *getName() const override { return "Transparent"; }

    // RenderGraph wiring
    void register_graph(class RenderGraph *graph,
                        RGImageHandle drawHandle,
                        RGImageHandle depthHandle);

private:
    void draw_transparent(VkCommandBuffer cmd,
                          EngineContext *context,
                          const class RGPassResources &resources,
                          RGImageHandle drawHandle,
                          RGImageHandle depthHandle) const;

    EngineContext *_context{};
};

