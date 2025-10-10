#pragma once

#include <core/vk_types.h>
#include <core/vk_descriptors.h>

class DeviceManager;

struct FrameResources
{
    VkSemaphore _swapchainSemaphore = VK_NULL_HANDLE;
    VkSemaphore _renderSemaphore = VK_NULL_HANDLE;
    VkFence _renderFence = VK_NULL_HANDLE;

    VkCommandPool _commandPool = VK_NULL_HANDLE;
    VkCommandBuffer _mainCommandBuffer = VK_NULL_HANDLE;

    DeletionQueue _deletionQueue;
    DescriptorAllocatorGrowable _frameDescriptors;

    void init(DeviceManager *deviceManager,
              std::span<DescriptorAllocatorGrowable::PoolSizeRatio> framePoolSizes);

    void cleanup(DeviceManager *deviceManager);
};
