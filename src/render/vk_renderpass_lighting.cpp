#include "vk_renderpass_lighting.h"

#include "frame_resources.h"
#include "vk_descriptor_manager.h"
#include "vk_device.h"
#include "core/engine_context.h"
#include "core/vk_initializers.h"
#include "core/vk_resource.h"
#include "render/vk_pipelines.h"
#include "core/vk_pipeline_manager.h"
#include "core/asset_manager.h"
#include "core/vk_descriptors.h"

#include "vk_mem_alloc.h"
#include "vk_sampler_manager.h"
#include "vk_swapchain.h"
#include "render/rg_graph.h"

void LightingPass::init(EngineContext *context)
{
    _context = context;

    // Build descriptor layout for GBuffer inputs
    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        _gBufferInputDescriptorLayout = builder.build(_context->getDevice()->device(), VK_SHADER_STAGE_FRAGMENT_BIT);
    }

    // Allocate and write GBuffer descriptor set
    _gBufferInputDescriptorSet = _context->getDescriptors()->allocate(
        _context->getDevice()->device(), _gBufferInputDescriptorLayout);
    {
        DescriptorWriter writer;
        writer.write_image(0, _context->getSwapchain()->gBufferPosition().imageView, _context->getSamplers()->defaultLinear(),
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        writer.write_image(1, _context->getSwapchain()->gBufferNormal().imageView, _context->getSamplers()->defaultLinear(),
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        writer.write_image(2, _context->getSwapchain()->gBufferAlbedo().imageView, _context->getSamplers()->defaultLinear(),
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        writer.update_set(_context->getDevice()->device(), _gBufferInputDescriptorSet);
    }

    // Shadow map descriptor layout (set = 2, updated per-frame)
    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        _shadowDescriptorLayout = builder.build(_context->getDevice()->device(), VK_SHADER_STAGE_FRAGMENT_BIT);
    }

    // Build lighting pipeline through PipelineManager
    VkDescriptorSetLayout layouts[] = {
        _context->getDescriptorLayouts()->gpuSceneDataLayout(),
        _gBufferInputDescriptorLayout,
        _shadowDescriptorLayout
    };

    GraphicsPipelineCreateInfo info{};
    info.vertexShaderPath = _context->getAssets()->shaderPath("fullscreen.vert.spv");
    info.fragmentShaderPath = _context->getAssets()->shaderPath("deferred_lighting.frag.spv");
    info.setLayouts.assign(std::begin(layouts), std::end(layouts));
    info.configure = [this](PipelineBuilder &b) {
        b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
        b.set_polygon_mode(VK_POLYGON_MODE_FILL);
        b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
        b.set_multisampling_none();
        b.enable_blending_alphablend();
        b.disable_depthtest();
        b.set_color_attachment_format(_context->getSwapchain()->drawImage().imageFormat);
    };
    _context->pipelines->createGraphicsPipeline("deferred_lighting", info);

    // fetch the handles so current frame uses latest versions
    MaterialPipeline mp{};
    _context->pipelines->getMaterialPipeline("deferred_lighting", mp);
    _pipeline = mp.pipeline;
    _pipelineLayout = mp.layout;

    _deletionQueue.push_function([&]() {
        // Pipelines are owned by PipelineManager; only destroy our local descriptor set layout
        vkDestroyDescriptorSetLayout(_context->getDevice()->device(), _gBufferInputDescriptorLayout, nullptr);
        vkDestroyDescriptorSetLayout(_context->getDevice()->device(), _shadowDescriptorLayout, nullptr);
    });
}

void LightingPass::execute(VkCommandBuffer)
{
    // Lighting is executed via the render graph now.
}

void LightingPass::register_graph(RenderGraph *graph,
                                  RGImageHandle drawHandle,
                                  RGImageHandle gbufferPosition,
                                  RGImageHandle gbufferNormal,
                                  RGImageHandle gbufferAlbedo,
                                  RGImageHandle shadowDepth)
{
    if (!graph || !drawHandle.valid() || !gbufferPosition.valid() || !gbufferNormal.valid() || !gbufferAlbedo.valid() || !shadowDepth.valid())
    {
        return;
    }

    graph->add_pass(
        "Lighting",
        RGPassType::Graphics,
        [drawHandle, gbufferPosition, gbufferNormal, gbufferAlbedo, shadowDepth](RGPassBuilder &builder, EngineContext *)
        {
            builder.read(gbufferPosition, RGImageUsage::SampledFragment);
            builder.read(gbufferNormal, RGImageUsage::SampledFragment);
            builder.read(gbufferAlbedo, RGImageUsage::SampledFragment);
            builder.read(shadowDepth, RGImageUsage::SampledFragment);

            builder.write_color(drawHandle);
        },
        [this, drawHandle, shadowDepth](VkCommandBuffer cmd, const RGPassResources &res, EngineContext *ctx)
        {
            draw_lighting(cmd, ctx, res, drawHandle, shadowDepth);
        });
}

void LightingPass::draw_lighting(VkCommandBuffer cmd,
                                 EngineContext *context,
                                 const RGPassResources &resources,
                                 RGImageHandle drawHandle,
                                 RGImageHandle shadowDepth)
{
    EngineContext *ctxLocal = context ? context : _context;
    if (!ctxLocal || !ctxLocal->currentFrame) return;

    ResourceManager *resourceManager = ctxLocal->getResources();
    DeviceManager *deviceManager = ctxLocal->getDevice();
    DescriptorManager *descriptorLayouts = ctxLocal->getDescriptorLayouts();
    PipelineManager *pipelineManager = ctxLocal->pipelines;
    if (!resourceManager || !deviceManager || !descriptorLayouts || !pipelineManager) return;

    VkImageView drawView = resources.image_view(drawHandle);
    if (drawView == VK_NULL_HANDLE) return;

    // Re-fetch pipeline in case it was hot-reloaded
    pipelineManager->getGraphics("deferred_lighting", _pipeline, _pipelineLayout);

    // Dynamic rendering is handled by the RenderGraph using the declared draw attachment.

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

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipelineLayout, 0, 1, &globalDescriptor, 0,
                            nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipelineLayout, 1, 1,
                            &_gBufferInputDescriptorSet, 0, nullptr);

    // Allocate and write shadow descriptor set for this frame (set = 2)
    VkDescriptorSet shadowSet = ctxLocal->currentFrame->_frameDescriptors.allocate(
        deviceManager->device(), _shadowDescriptorLayout);
    {
        VkImageView shadowView = resources.image_view(shadowDepth);
        DescriptorWriter writer2;
        writer2.write_image(0, shadowView, ctxLocal->getSamplers()->defaultLinear(),
                            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                            VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        writer2.update_set(deviceManager->device(), shadowSet);
    }
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipelineLayout, 2, 1, &shadowSet, 0, nullptr);

    VkViewport viewport{};
    viewport.x = 0;
    viewport.y = 0;
    viewport.width = static_cast<float>(ctxLocal->getDrawExtent().width);
    viewport.height = static_cast<float>(ctxLocal->getDrawExtent().height);
    viewport.minDepth = 0.f;
    viewport.maxDepth = 1.f;
    vkCmdSetViewport(cmd, 0, 1, &viewport);

    VkRect2D scissor{};
    scissor.offset = {0, 0};
    scissor.extent = {ctxLocal->getDrawExtent().width, ctxLocal->getDrawExtent().height};
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    vkCmdDraw(cmd, 3, 1, 0, 0);

    // RenderGraph ends rendering.
}

void LightingPass::cleanup()
{
    _deletionQueue.flush();
    fmt::print("LightingPass::cleanup()\n");
}
