#include "vk_swapchain.h"

#include <SDL_video.h>

#include "vk_device.h"
#include "vk_initializers.h"
#include "vk_resource.h"

void SwapchainManager::init_swapchain()
{
    create_swapchain(_windowExtent.width, _windowExtent.height);

    // Create images used across the frame (draw, depth, GBuffer)
    // Split to helper so we can reuse on resize
    // (Definition added below)
    //
    // On creation we also push a cleanup lambda to _deletionQueue for final shutdown.
    // On resize we will flush that queue first to destroy previous resources.

    // depth/draw/gbuffer sized to current window extent
    auto create_frame_images = [this]() {
        VkExtent3D drawImageExtent = { _windowExtent.width, _windowExtent.height, 1 };

        // Draw HDR target
        _drawImage.imageFormat = VK_FORMAT_R16G16B16A16_SFLOAT;
        _drawImage.imageExtent = drawImageExtent;

        VkImageUsageFlags drawImageUsages{};
        drawImageUsages |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
        drawImageUsages |= VK_IMAGE_USAGE_STORAGE_BIT;
        drawImageUsages |= VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

        VkImageCreateInfo rimg_info = vkinit::image_create_info(_drawImage.imageFormat, drawImageUsages, drawImageExtent);

        VmaAllocationCreateInfo rimg_allocinfo = {};
        rimg_allocinfo.usage = VMA_MEMORY_USAGE_GPU_ONLY;
        rimg_allocinfo.requiredFlags = static_cast<VkMemoryPropertyFlags>(VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        vmaCreateImage(_deviceManager->allocator(), &rimg_info, &rimg_allocinfo,
                       &_drawImage.image, &_drawImage.allocation, nullptr);

        VkImageViewCreateInfo rview_info = vkinit::imageview_create_info(_drawImage.imageFormat, _drawImage.image,
                                                                         VK_IMAGE_ASPECT_COLOR_BIT);
        VK_CHECK(vkCreateImageView(_deviceManager->device(), &rview_info, nullptr, &_drawImage.imageView));

        // Depth
        _depthImage.imageFormat = VK_FORMAT_D32_SFLOAT;
        _depthImage.imageExtent = drawImageExtent;
        VkImageUsageFlags depthImageUsages{};
        depthImageUsages |= VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
        VkImageCreateInfo dimg_info = vkinit::image_create_info(_depthImage.imageFormat, depthImageUsages, drawImageExtent);
        vmaCreateImage(_deviceManager->allocator(), &dimg_info, &rimg_allocinfo, &_depthImage.image,
                       &_depthImage.allocation, nullptr);
        VkImageViewCreateInfo dview_info = vkinit::imageview_create_info(_depthImage.imageFormat, _depthImage.image,
                                                                         VK_IMAGE_ASPECT_DEPTH_BIT);
        VK_CHECK(vkCreateImageView(_deviceManager->device(), &dview_info, nullptr, &_depthImage.imageView));

        // GBuffer (SRGB not used to keep linear lighting)
        _gBufferPosition = _resourceManager->create_image(drawImageExtent, VK_FORMAT_R16G16B16A16_SFLOAT,
                                                          VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
        _gBufferNormal = _resourceManager->create_image(drawImageExtent, VK_FORMAT_R16G16B16A16_SFLOAT,
                                                        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
        _gBufferAlbedo = _resourceManager->create_image(drawImageExtent, VK_FORMAT_R8G8B8A8_UNORM,
                                                        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);

        _deletionQueue.push_function([=]() {
            vkDestroyImageView(_deviceManager->device(), _drawImage.imageView, nullptr);
            vmaDestroyImage(_deviceManager->allocator(), _drawImage.image, _drawImage.allocation);

            vkDestroyImageView(_deviceManager->device(), _depthImage.imageView, nullptr);
            vmaDestroyImage(_deviceManager->allocator(), _depthImage.image, _depthImage.allocation);

            _resourceManager->destroy_image(_gBufferPosition);
            _resourceManager->destroy_image(_gBufferNormal);
            _resourceManager->destroy_image(_gBufferAlbedo);
        });
    };

    create_frame_images();
}

void SwapchainManager::cleanup()
{
    _deletionQueue.flush();
    destroy_swapchain();
    fmt::print("SwapchainManager::cleanup()\n");
}

void SwapchainManager::create_swapchain(uint32_t width, uint32_t height)
{
    vkb::SwapchainBuilder swapchainBuilder{
        _deviceManager->physicalDevice(), _deviceManager->device(), _deviceManager->surface()
    };

    _swapchainImageFormat = VK_FORMAT_B8G8R8A8_UNORM;

    vkb::Swapchain vkbSwapchain = swapchainBuilder
            //.use_default_format_selection()
            .set_desired_format(VkSurfaceFormatKHR{
                .format = _swapchainImageFormat, .colorSpace = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR
            })
            //use vsync present mode
            .set_desired_present_mode(VK_PRESENT_MODE_FIFO_KHR)
            .set_desired_extent(width, height)
            .add_image_usage_flags(VK_IMAGE_USAGE_TRANSFER_DST_BIT)
            .build()
            .value();

    _swapchainExtent = vkbSwapchain.extent;
    //store swapchain and its related images
    _swapchain = vkbSwapchain.swapchain;
    _swapchainImages = vkbSwapchain.get_images().value();
    _swapchainImageViews = vkbSwapchain.get_image_views().value();
}

void SwapchainManager::destroy_swapchain() const
{
    vkDestroySwapchainKHR(_deviceManager->device(), _swapchain, nullptr);

    for (auto _swapchainImageView: _swapchainImageViews)
    {
        vkDestroyImageView(_deviceManager->device(), _swapchainImageView, nullptr);
    }
}

void SwapchainManager::resize_swapchain(struct SDL_Window *window)
{
    vkDeviceWaitIdle(_deviceManager->device());

    destroy_swapchain();

    // Destroy per-frame images before recreating them
    _deletionQueue.flush();

    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    _windowExtent.width = w;
    _windowExtent.height = h;

    create_swapchain(_windowExtent.width, _windowExtent.height);

    // Recreate frame images at the new size
    // (duplicate the same logic used at init time)
    VkExtent3D drawImageExtent = { _windowExtent.width, _windowExtent.height, 1 };

    _drawImage.imageFormat = VK_FORMAT_R16G16B16A16_SFLOAT;
    _drawImage.imageExtent = drawImageExtent;

    VkImageUsageFlags drawImageUsages{};
    drawImageUsages |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    drawImageUsages |= VK_IMAGE_USAGE_STORAGE_BIT;
    drawImageUsages |= VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

    VkImageCreateInfo rimg_info = vkinit::image_create_info(_drawImage.imageFormat, drawImageUsages, drawImageExtent);
    VmaAllocationCreateInfo rimg_allocinfo = {};
    rimg_allocinfo.usage = VMA_MEMORY_USAGE_GPU_ONLY;
    rimg_allocinfo.requiredFlags = static_cast<VkMemoryPropertyFlags>(VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    vmaCreateImage(_deviceManager->allocator(), &rimg_info, &rimg_allocinfo, &_drawImage.image, &_drawImage.allocation,
                   nullptr);

    VkImageViewCreateInfo rview_info = vkinit::imageview_create_info(_drawImage.imageFormat, _drawImage.image,
                                                                     VK_IMAGE_ASPECT_COLOR_BIT);
    VK_CHECK(vkCreateImageView(_deviceManager->device(), &rview_info, nullptr, &_drawImage.imageView));

    _depthImage.imageFormat = VK_FORMAT_D32_SFLOAT;
    _depthImage.imageExtent = drawImageExtent;
    VkImageUsageFlags depthImageUsages{};
    depthImageUsages |= VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    VkImageCreateInfo dimg_info = vkinit::image_create_info(_depthImage.imageFormat, depthImageUsages, drawImageExtent);
    vmaCreateImage(_deviceManager->allocator(), &dimg_info, &rimg_allocinfo, &_depthImage.image,
                   &_depthImage.allocation, nullptr);

    VkImageViewCreateInfo dview_info = vkinit::imageview_create_info(_depthImage.imageFormat, _depthImage.image,
                                                                     VK_IMAGE_ASPECT_DEPTH_BIT);
    VK_CHECK(vkCreateImageView(_deviceManager->device(), &dview_info, nullptr, &_depthImage.imageView));

    _gBufferPosition = _resourceManager->create_image(drawImageExtent, VK_FORMAT_R16G16B16A16_SFLOAT,
                                                      VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
    _gBufferNormal = _resourceManager->create_image(drawImageExtent, VK_FORMAT_R16G16B16A16_SFLOAT,
                                                    VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
    _gBufferAlbedo = _resourceManager->create_image(drawImageExtent, VK_FORMAT_R8G8B8A8_UNORM,
                                                    VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);

    _deletionQueue.push_function([=]() {
        vkDestroyImageView(_deviceManager->device(), _drawImage.imageView, nullptr);
        vmaDestroyImage(_deviceManager->allocator(), _drawImage.image, _drawImage.allocation);

        vkDestroyImageView(_deviceManager->device(), _depthImage.imageView, nullptr);
        vmaDestroyImage(_deviceManager->allocator(), _depthImage.image, _depthImage.allocation);

        _resourceManager->destroy_image(_gBufferPosition);
        _resourceManager->destroy_image(_gBufferNormal);
        _resourceManager->destroy_image(_gBufferAlbedo);
    });

    resize_requested = false;
}
