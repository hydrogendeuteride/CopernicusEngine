#include "vk_descriptor_manager.h"
#include "vk_device.h"
#include "vk_descriptors.h"

void DescriptorManager::init(DeviceManager *deviceManager)
{
    _deviceManager = deviceManager;

    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        _singleImageDescriptorLayout = builder.build(_deviceManager->device(), VK_SHADER_STAGE_FRAGMENT_BIT);
    } {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
        _gpuSceneDataDescriptorLayout = builder.build(
            _deviceManager->device(), VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT);
    }
}

void DescriptorManager::cleanup()
{
    if (!_deviceManager) return;

    if (_singleImageDescriptorLayout)
    {
        vkDestroyDescriptorSetLayout(_deviceManager->device(), _singleImageDescriptorLayout, nullptr);
        _singleImageDescriptorLayout = VK_NULL_HANDLE;
    }
    if (_gpuSceneDataDescriptorLayout)
    {
        vkDestroyDescriptorSetLayout(_deviceManager->device(), _gpuSceneDataDescriptorLayout, nullptr);
        _gpuSceneDataDescriptorLayout = VK_NULL_HANDLE;
    }
}
