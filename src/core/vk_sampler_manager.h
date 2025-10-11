#pragma once

#include <core/vk_types.h>

class DeviceManager;

class SamplerManager
{
public:
    void init(DeviceManager *deviceManager);

    void cleanup();

    VkSampler defaultLinear() const { return _defaultSamplerLinear; }
    VkSampler defaultNearest() const { return _defaultSamplerNearest; }
    VkSampler shadowLinearClamp() const { return _shadowLinearClamp; }

private:
    DeviceManager *_deviceManager = nullptr;
    VkSampler _defaultSamplerLinear = VK_NULL_HANDLE;
    VkSampler _defaultSamplerNearest = VK_NULL_HANDLE;
    VkSampler _shadowLinearClamp = VK_NULL_HANDLE;
};
