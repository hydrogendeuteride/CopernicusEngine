#include "vk_materials.h"

#include "core/vk_engine.h"
#include "render/vk_pipelines.h"
#include "core/vk_initializers.h"
#include "core/vk_pipeline_manager.h"
#include "core/asset_manager.h"

namespace vkutil { bool load_shader_module(const char*, VkDevice, VkShaderModule*); }

void GLTFMetallic_Roughness::build_pipelines(VulkanEngine *engine)
{
    VkPushConstantRange matrixRange{};
    matrixRange.offset = 0;
    matrixRange.size = sizeof(GPUDrawPushConstants);
    matrixRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

    DescriptorLayoutBuilder layoutBuilder;
    layoutBuilder.add_binding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
    layoutBuilder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    layoutBuilder.add_binding(2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    materialLayout = layoutBuilder.build(engine->_deviceManager->device(),
                                         VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT);

    VkDescriptorSetLayout layouts[] = {
        engine->_descriptorManager->gpuSceneDataLayout(),
        materialLayout
    };

    // Register pipelines with the central PipelineManager
    GraphicsPipelineCreateInfo opaqueInfo{};
    opaqueInfo.vertexShaderPath = engine->_context->getAssets()->shaderPath("mesh.vert.spv");
    opaqueInfo.fragmentShaderPath = engine->_context->getAssets()->shaderPath("mesh.frag.spv");
    opaqueInfo.setLayouts.assign(std::begin(layouts), std::end(layouts));
    opaqueInfo.pushConstants = {matrixRange};
    opaqueInfo.configure = [engine](PipelineBuilder &b) {
        b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
        b.set_polygon_mode(VK_POLYGON_MODE_FILL);
        b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
        b.set_multisampling_none();
        b.disable_blending();
        // Reverse-Z depth test configuration
        b.enable_depthtest(true, VK_COMPARE_OP_GREATER_OR_EQUAL);
        b.set_color_attachment_format(engine->_swapchainManager->drawImage().imageFormat);
        b.set_depth_format(engine->_swapchainManager->depthImage().imageFormat);
    };
    engine->_pipelineManager->registerGraphics("mesh.opaque", opaqueInfo);

    GraphicsPipelineCreateInfo transparentInfo = opaqueInfo;
    transparentInfo.configure = [engine](PipelineBuilder &b) {
        b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
        b.set_polygon_mode(VK_POLYGON_MODE_FILL);
        b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
        b.set_multisampling_none();
        // Physically-based transparency uses standard alpha blending
        b.enable_blending_alphablend();
        // Transparent pass: keep reverse-Z test (no writes)
        b.enable_depthtest(false, VK_COMPARE_OP_GREATER_OR_EQUAL);
        b.set_color_attachment_format(engine->_swapchainManager->drawImage().imageFormat);
        b.set_depth_format(engine->_swapchainManager->depthImage().imageFormat);
    };
    engine->_pipelineManager->registerGraphics("mesh.transparent", transparentInfo);

    GraphicsPipelineCreateInfo gbufferInfo{};
    gbufferInfo.vertexShaderPath = engine->_context->getAssets()->shaderPath("mesh.vert.spv");
    gbufferInfo.fragmentShaderPath = engine->_context->getAssets()->shaderPath("gbuffer.frag.spv");
    gbufferInfo.setLayouts.assign(std::begin(layouts), std::end(layouts));
    gbufferInfo.pushConstants = {matrixRange};
    gbufferInfo.configure = [engine](PipelineBuilder &b) {
        b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
        b.set_polygon_mode(VK_POLYGON_MODE_FILL);
        b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
        b.set_multisampling_none();
        b.disable_blending();
        // GBuffer uses reverse-Z depth
        b.enable_depthtest(true, VK_COMPARE_OP_GREATER_OR_EQUAL);
        VkFormat gFormats[] = {
            engine->_swapchainManager->gBufferPosition().imageFormat,
            engine->_swapchainManager->gBufferNormal().imageFormat,
            engine->_swapchainManager->gBufferAlbedo().imageFormat
        };
        b.set_color_attachment_formats(std::span<VkFormat>(gFormats, 3));
        b.set_depth_format(engine->_swapchainManager->depthImage().imageFormat);
    };
    engine->_pipelineManager->registerGraphics("mesh.gbuffer", gbufferInfo);

    engine->_pipelineManager->getMaterialPipeline("mesh.opaque", opaquePipeline);
    engine->_pipelineManager->getMaterialPipeline("mesh.transparent", transparentPipeline);
    engine->_pipelineManager->getMaterialPipeline("mesh.gbuffer", gBufferPipeline);
}

void GLTFMetallic_Roughness::clear_resources(VkDevice device) const
{
    vkDestroyDescriptorSetLayout(device, materialLayout, nullptr);
}

MaterialInstance GLTFMetallic_Roughness::write_material(VkDevice device, MaterialPass pass,
                                                        const MaterialResources &resources,
                                                        DescriptorAllocatorGrowable &descriptorAllocator)
{
    MaterialInstance matData{};
    matData.passType = pass;
    if (pass == MaterialPass::Transparent)
    {
        matData.pipeline = &transparentPipeline;
    }
    else
    {
        matData.pipeline = &gBufferPipeline;
    }

    matData.materialSet = descriptorAllocator.allocate(device, materialLayout);

    writer.clear();
    writer.write_buffer(0, resources.dataBuffer, sizeof(MaterialConstants), resources.dataBufferOffset,
                        VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
    writer.write_image(1, resources.colorImage.imageView, resources.colorSampler,
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(2, resources.metalRoughImage.imageView, resources.metalRoughSampler,
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    writer.update_set(device, matData.materialSet);

    return matData;
}
