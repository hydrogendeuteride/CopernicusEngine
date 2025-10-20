#include "vk_sampler_manager.h"
#include "vk_device.h"

void SamplerManager::init(DeviceManager *deviceManager)
{
    _deviceManager = deviceManager;

    VkSamplerCreateInfo sampl{.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};

    // Sensible, cross-vendor defaults
    sampl.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sampl.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sampl.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sampl.mipmapMode   = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    sampl.minLod       = 0.0f;
    sampl.maxLod       = VK_LOD_CLAMP_NONE;
    sampl.mipLodBias   = 0.0f;
    sampl.anisotropyEnable = VK_FALSE; // set true + maxAnisotropy if feature enabled
    sampl.borderColor  = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
    sampl.unnormalizedCoordinates = VK_FALSE;

    // Nearest defaults
    sampl.magFilter = VK_FILTER_NEAREST;
    sampl.minFilter = VK_FILTER_NEAREST;
    vkCreateSampler(_deviceManager->device(), &sampl, nullptr, &_defaultSamplerNearest);

    // Linear defaults
    sampl.magFilter = VK_FILTER_LINEAR;
    sampl.minFilter = VK_FILTER_LINEAR;
    vkCreateSampler(_deviceManager->device(), &sampl, nullptr, &_defaultSamplerLinear);

    // Shadow linear clamp sampler (border=white)
    VkSamplerCreateInfo sh = sampl;
    sh.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    sh.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    sh.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    sh.compareEnable = VK_FALSE; // manual PCF
    // Depth shadow maps are single-level; keep base LOD only and avoid mip filtering.
    sh.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    sh.maxLod = 0.0f;
    sh.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
    vkCreateSampler(_deviceManager->device(), &sh, nullptr, &_shadowLinearClamp);
}

void SamplerManager::cleanup()
{
    if (!_deviceManager) return;

    if (_defaultSamplerNearest)
    {
        vkDestroySampler(_deviceManager->device(), _defaultSamplerNearest, nullptr);
        _defaultSamplerNearest = VK_NULL_HANDLE;
    }
    if (_defaultSamplerLinear)
    {
        vkDestroySampler(_deviceManager->device(), _defaultSamplerLinear, nullptr);
        _defaultSamplerLinear = VK_NULL_HANDLE;
    }
    if (_shadowLinearClamp)
    {
        vkDestroySampler(_deviceManager->device(), _shadowLinearClamp, nullptr);
        _shadowLinearClamp = VK_NULL_HANDLE;
    }
}
