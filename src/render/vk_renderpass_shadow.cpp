#include "vk_renderpass_shadow.h"

#include <unordered_set>
#include <string>

#include "core/engine_context.h"
#include "render/rg_graph.h"
#include "render/rg_builder.h"
#include "vk_swapchain.h"
#include "vk_scene.h"
#include "frame_resources.h"
#include "vk_descriptor_manager.h"
#include "vk_device.h"
#include "vk_resource.h"
#include "core/vk_initializers.h"
#include "core/vk_pipeline_manager.h"
#include "core/asset_manager.h"
#include "render/vk_pipelines.h"
#include "core/vk_types.h"

void ShadowPass::init(EngineContext *context)
{
    _context = context;

    if (!_context || !_context->pipelines) return;

    // Build a depth-only graphics pipeline for shadow map rendering
    // Keep push constants matching current shader layout for now
    VkPushConstantRange pc{};
    pc.offset = 0;
    // Push constants layout in shadow.vert is mat4 + device address + uint, rounded to 16 bytes
    const uint32_t pcRaw = static_cast<uint32_t>(sizeof(GPUDrawPushConstants) + sizeof(uint32_t));
    const uint32_t pcAligned = (pcRaw + 15u) & ~15u; // 16-byte alignment to match std430 expectations
    pc.size = pcAligned;
    pc.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

    GraphicsPipelineCreateInfo info{};
    info.vertexShaderPath = _context->getAssets()->shaderPath("shadow.vert.spv");
    info.fragmentShaderPath = _context->getAssets()->shaderPath("shadow.frag.spv");
    info.setLayouts = { _context->getDescriptorLayouts()->gpuSceneDataLayout() };
    info.pushConstants = { pc };
    info.configure = [this](PipelineBuilder &b) {
        b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
        b.set_polygon_mode(VK_POLYGON_MODE_FILL);
        b.set_cull_mode(VK_CULL_MODE_BACK_BIT, VK_FRONT_FACE_CLOCKWISE);
        b.set_multisampling_none();
        b.disable_blending();
        // Reverse-Z depth test for shadow maps (clear=0.0, GREATER_OR_EQUAL)
        b.enable_depthtest(true, VK_COMPARE_OP_GREATER_OR_EQUAL);
        b.set_depth_format(VK_FORMAT_D32_SFLOAT);

        // Static depth bias to help with surface acne (tune later)
        b._rasterizer.depthBiasEnable = VK_TRUE;
        b._rasterizer.depthBiasConstantFactor = 2.0f;
        b._rasterizer.depthBiasSlopeFactor = 2.0f;
        b._rasterizer.depthBiasClamp = 0.0f;
    };

    _context->pipelines->createGraphicsPipeline("mesh.shadow", info);
}

void ShadowPass::cleanup()
{
    // Nothing yet; pipelines and descriptors will be added later
    fmt::print("ShadowPass::cleanup()\n");
}

void ShadowPass::execute(VkCommandBuffer)
{
    // Shadow rendering is done via the RenderGraph registration.
}

void ShadowPass::register_graph(RenderGraph *graph, std::span<RGImageHandle> cascades, VkExtent2D extent)
{
    if (!graph || cascades.empty()) return;

    for (uint32_t i = 0; i < cascades.size(); ++i)
    {
        RGImageHandle shadowDepth = cascades[i];
        if (!shadowDepth.valid()) continue;

        std::string passName = std::string("ShadowMap[") + std::to_string(i) + "]";
        graph->add_pass(
            passName.c_str(),
            RGPassType::Graphics,
            [shadowDepth](RGPassBuilder &builder, EngineContext *ctx)
            {
                // Reverse-Z depth clear to 0.0
                VkClearValue clear{}; clear.depthStencil = {0.f, 0};
                builder.write_depth(shadowDepth, true, clear);

                // Ensure index/vertex buffers are tracked as reads (like Geometry)
                if (ctx)
                {
                    const DrawContext &dc = ctx->getMainDrawContext();
                    std::unordered_set<VkBuffer> indexSet;
                    std::unordered_set<VkBuffer> vertexSet;
                    auto collect = [&](const std::vector<RenderObject> &v)
                    {
                        for (const auto &r : v)
                        {
                            if (r.indexBuffer) indexSet.insert(r.indexBuffer);
                            if (r.vertexBuffer) vertexSet.insert(r.vertexBuffer);
                        }
                    };
                    collect(dc.OpaqueSurfaces);
                    // Ignore transparent for shadow map

                    for (VkBuffer b : indexSet)
                        builder.read_buffer(b, RGBufferUsage::IndexRead, 0, "shadow.index");
                    for (VkBuffer b : vertexSet)
                        builder.read_buffer(b, RGBufferUsage::StorageRead, 0, "shadow.vertex");
                }
            },
            [this, shadowDepth, extent, i](VkCommandBuffer cmd, const RGPassResources &res, EngineContext *ctx)
            {
                draw_shadow(cmd, ctx, res, shadowDepth, extent, i);
            });
    }
}

void ShadowPass::draw_shadow(VkCommandBuffer cmd,
                             EngineContext *context,
                             const RGPassResources &/*resources*/,
                             RGImageHandle /*shadowDepth*/,
                             VkExtent2D extent,
                             uint32_t cascadeIndex) const
{
    EngineContext *ctxLocal = context ? context : _context;
    if (!ctxLocal || !ctxLocal->currentFrame) return;

    ResourceManager *resourceManager = ctxLocal->getResources();
    DeviceManager *deviceManager = ctxLocal->getDevice();
    DescriptorManager *descriptorLayouts = ctxLocal->getDescriptorLayouts();
    PipelineManager *pipelineManager = ctxLocal->pipelines;
    if (!resourceManager || !deviceManager || !descriptorLayouts || !pipelineManager) return;

    VkPipeline pipeline{}; VkPipelineLayout layout{};
    if (!pipelineManager->getGraphics("mesh.shadow", pipeline, layout)) return;

    // Create and upload per-pass scene UBO
    AllocatedBuffer gpuSceneDataBuffer = resourceManager->create_buffer(
        sizeof(GPUSceneData), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
        VMA_MEMORY_USAGE_CPU_TO_GPU);
    ctxLocal->currentFrame->_deletionQueue.push_function([resourceManager, gpuSceneDataBuffer]()
    {
        resourceManager->destroy_buffer(gpuSceneDataBuffer);
    });

    VmaAllocationInfo allocInfo{};
    vmaGetAllocationInfo(deviceManager->allocator(), gpuSceneDataBuffer.allocation, &allocInfo);
    auto *sceneUniformData = static_cast<GPUSceneData *>(allocInfo.pMappedData);
    *sceneUniformData = ctxLocal->getSceneData();
    vmaFlushAllocation(deviceManager->allocator(), gpuSceneDataBuffer.allocation, 0, sizeof(GPUSceneData));

    VkDescriptorSet globalDescriptor = ctxLocal->currentFrame->_frameDescriptors.allocate(
        deviceManager->device(), descriptorLayouts->gpuSceneDataLayout());
    DescriptorWriter writer;
    writer.write_buffer(0, gpuSceneDataBuffer.buffer, sizeof(GPUSceneData), 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
    writer.update_set(deviceManager->device(), globalDescriptor);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, 1, &globalDescriptor, 0, nullptr);

    VkViewport viewport{};
    viewport.x = 0;
    viewport.y = 0;
    viewport.width = static_cast<float>(extent.width);
    viewport.height = static_cast<float>(extent.height);
    viewport.minDepth = 0.f;
    viewport.maxDepth = 1.f;
    vkCmdSetViewport(cmd, 0, 1, &viewport);

    VkRect2D scissor{};
    scissor.offset = {0, 0};
    scissor.extent = extent;
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    const DrawContext &dc = ctxLocal->getMainDrawContext();

    VkBuffer lastIndexBuffer = VK_NULL_HANDLE;

    struct ShadowPC
    {
        GPUDrawPushConstants draw;
        uint32_t cascadeIndex;
    };
    for (const auto &r : dc.OpaqueSurfaces)
    {
        if (r.indexBuffer != lastIndexBuffer)
        {
            lastIndexBuffer = r.indexBuffer;
            vkCmdBindIndexBuffer(cmd, r.indexBuffer, 0, VK_INDEX_TYPE_UINT32);
        }

        ShadowPC spc{};
        spc.draw.worldMatrix = r.transform;
        spc.draw.vertexBuffer = r.vertexBufferAddress;
        spc.cascadeIndex = cascadeIndex;
        vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(ShadowPC), &spc);
        vkCmdDrawIndexed(cmd, r.indexCount, 1, r.firstIndex, 0, 0);
    }
}
