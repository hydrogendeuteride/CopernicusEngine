#pragma once

#include <core/vk_types.h>
#include <core/vk_descriptors.h>

#include "vk_device.h"

class DeviceManager;

class DescriptorManager
{
public:
    void init(DeviceManager *deviceManager);

    void cleanup();

    VkDescriptorSetLayout gpuSceneDataLayout() const { return _gpuSceneDataDescriptorLayout; }
    VkDescriptorSetLayout singleImageLayout() const { return _singleImageDescriptorLayout; }

private:
    DeviceManager *_deviceManager = nullptr;
    VkDescriptorSetLayout _singleImageDescriptorLayout = VK_NULL_HANDLE;
    VkDescriptorSetLayout _gpuSceneDataDescriptorLayout = VK_NULL_HANDLE;
};
