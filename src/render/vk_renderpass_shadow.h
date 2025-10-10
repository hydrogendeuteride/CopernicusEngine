#pragma once

#include "vk_renderpass.h"
#include <render/rg_types.h>

class RenderGraph;
class EngineContext;
class RGPassResources;

// Depth-only directional shadow map pass (skeleton)
// - Writes a depth image using reversed-Z (clear=0)
// - Draw function will be filled in a later step
class ShadowPass : public IRenderPass
{
public:
    void init(EngineContext *context) override;
    void cleanup() override;
    void execute(VkCommandBuffer cmd) override;

    const char *getName() const override { return "ShadowMap"; }

    // Register the depth-only pass into the render graph
    void register_graph(RenderGraph *graph, RGImageHandle shadowDepth, VkExtent2D extent);

private:
    EngineContext *_context = nullptr;

    void draw_shadow(VkCommandBuffer cmd,
                     EngineContext *context,
                     const RGPassResources &resources,
                     RGImageHandle shadowDepth,
                     VkExtent2D extent) const;
};
