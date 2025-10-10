#pragma once
#include "vk_renderpass.h"
#include <render/rg_types.h>

class SwapchainManager;
class RenderGraph;

class GeometryPass : public IRenderPass
{
public:
    void init(EngineContext *context) override;
    void cleanup() override;
    void execute(VkCommandBuffer cmd) override;

    const char *getName() const override { return "Geometry"; }

    void register_graph(RenderGraph *graph,
                        RGImageHandle gbufferPosition,
                        RGImageHandle gbufferNormal,
                        RGImageHandle gbufferAlbedo,
                        RGImageHandle depthHandle);

private:
    EngineContext *_context = nullptr;

    void draw_geometry(VkCommandBuffer cmd,
                       EngineContext *context,
                       const class RGPassResources &resources,
                       RGImageHandle gbufferPosition,
                       RGImageHandle gbufferNormal,
                       RGImageHandle gbufferAlbedo,
                       RGImageHandle depthHandle) const;
};
