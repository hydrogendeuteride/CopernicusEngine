#include "vk_resource.h"
#include "vk_device.h"
#include "vk_images.h"
#include "vk_initializers.h"

#include "vk_mem_alloc.h"
#include <render/rg_graph.h>
#include <render/rg_builder.h>
#include <render/rg_resources.h>

#include "frame_resources.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

void ResourceManager::init(DeviceManager *deviceManager)
{
    _deviceManager = deviceManager;

    VkCommandPoolCreateInfo commandPoolInfo = vkinit::command_pool_create_info(
        _deviceManager->graphicsQueueFamily(),
        VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT
    );

    VK_CHECK(vkCreateCommandPool(_deviceManager->device(), &commandPoolInfo, nullptr, &_immCommandPool));

    VkCommandBufferAllocateInfo cmdAllocInfo = vkinit::command_buffer_allocate_info(_immCommandPool, 1);
    VK_CHECK(vkAllocateCommandBuffers(_deviceManager->device(), &cmdAllocInfo, &_immCommandBuffer));

    VkFenceCreateInfo fenceCreateInfo = vkinit::fence_create_info(VK_FENCE_CREATE_SIGNALED_BIT);
    VK_CHECK(vkCreateFence(_deviceManager->device(), &fenceCreateInfo, nullptr, &_immFence));

    _deletionQueue.push_function([=]() {
        vkDestroyCommandPool(_deviceManager->device(), _immCommandPool, nullptr);
        vkDestroyFence(_deviceManager->device(), _immFence, nullptr);
    });
}

AllocatedBuffer ResourceManager::create_buffer(size_t allocSize, VkBufferUsageFlags usage,
                                               VmaMemoryUsage memoryUsage) const
{
    VkBufferCreateInfo bufferInfo = {.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bufferInfo.pNext = nullptr;
    bufferInfo.size = allocSize;
    bufferInfo.usage = usage;

    VmaAllocationCreateInfo vmaallocInfo = {};
    vmaallocInfo.usage = memoryUsage;
    vmaallocInfo.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;

    AllocatedBuffer newBuffer{};
    VK_CHECK(vmaCreateBuffer(_deviceManager->allocator(), &bufferInfo, &vmaallocInfo,
        &newBuffer.buffer, &newBuffer.allocation, &newBuffer.info));

    return newBuffer;
}

void ResourceManager::immediate_submit(std::function<void(VkCommandBuffer)> &&function) const
{
    VK_CHECK(vkResetFences(_deviceManager->device(), 1, &_immFence));
    VK_CHECK(vkResetCommandBuffer(_immCommandBuffer, 0));

    VkCommandBuffer cmd = _immCommandBuffer;
    VkCommandBufferBeginInfo cmdBeginInfo = vkinit::command_buffer_begin_info(
        VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);

    VK_CHECK(vkBeginCommandBuffer(cmd, &cmdBeginInfo));
    function(cmd);
    VK_CHECK(vkEndCommandBuffer(cmd));

    VkCommandBufferSubmitInfo cmdinfo = vkinit::command_buffer_submit_info(cmd);
    VkSubmitInfo2 submit = vkinit::submit_info(&cmdinfo, nullptr, nullptr);

    VK_CHECK(vkQueueSubmit2(_deviceManager->graphicsQueue(), 1, &submit, _immFence));
    VK_CHECK(vkWaitForFences(_deviceManager->device(), 1, &_immFence, true, 9999999999));
}

void ResourceManager::destroy_buffer(const AllocatedBuffer &buffer) const
{
    vmaDestroyBuffer(_deviceManager->allocator(), buffer.buffer, buffer.allocation);
}

void ResourceManager::cleanup()
{
    fmt::print("ResourceManager::cleanup()\n");
    clear_pending_uploads();
    _deletionQueue.flush();
}

AllocatedImage ResourceManager::create_image(VkExtent3D size, VkFormat format, VkImageUsageFlags usage,
                                             bool mipmapped) const
{
    AllocatedImage newImage{};
    newImage.imageFormat = format;
    newImage.imageExtent = size;

    VkImageCreateInfo img_info = vkinit::image_create_info(format, usage, size);
    if (mipmapped)
    {
        img_info.mipLevels = static_cast<uint32_t>(std::floor(std::log2(std::max(size.width, size.height)))) + 1;
    }

    // always allocate images on dedicated GPU memory
    VmaAllocationCreateInfo allocinfo = {};
    allocinfo.usage = VMA_MEMORY_USAGE_GPU_ONLY;
    allocinfo.requiredFlags = static_cast<VkMemoryPropertyFlags>(VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    // allocate and create the image
    VK_CHECK(
        vmaCreateImage(_deviceManager->allocator(), &img_info, &allocinfo, &newImage.image, &newImage.allocation,
            nullptr));

    // if the format is a depth format, we will need to have it use the correct
    // aspect flag
    VkImageAspectFlags aspectFlag = VK_IMAGE_ASPECT_COLOR_BIT;
    if (format == VK_FORMAT_D32_SFLOAT)
    {
        aspectFlag = VK_IMAGE_ASPECT_DEPTH_BIT;
    }

    // build a image-view for the image
    VkImageViewCreateInfo view_info = vkinit::imageview_create_info(format, newImage.image, aspectFlag);
    view_info.subresourceRange.levelCount = img_info.mipLevels;

    VK_CHECK(vkCreateImageView(_deviceManager->device(), &view_info, nullptr, &newImage.imageView));

    return newImage;
}

AllocatedImage ResourceManager::create_image(const void *data, VkExtent3D size, VkFormat format,
                                             VkImageUsageFlags usage,
                                             bool mipmapped)
{
    size_t data_size = size.depth * size.width * size.height * 4;
    AllocatedBuffer uploadbuffer = create_buffer(data_size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                 VMA_MEMORY_USAGE_CPU_TO_GPU);

    memcpy(uploadbuffer.info.pMappedData, data, data_size);

    vmaFlushAllocation(_deviceManager->allocator(), uploadbuffer.allocation, 0, data_size);

    AllocatedImage new_image = create_image(size, format,
                                            usage | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
                                            mipmapped);

    PendingImageUpload pending{};
    pending.staging = uploadbuffer;
    pending.image = new_image.image;
    pending.extent = size;
    pending.format = format;
    pending.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    pending.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    pending.generateMips = mipmapped;

    _pendingImageUploads.push_back(std::move(pending));

    if (!_deferUploads)
    {
        process_queued_uploads_immediate();
    }

    return new_image;
}

void ResourceManager::destroy_image(const AllocatedImage &img) const
{
    vkDestroyImageView(_deviceManager->device(), img.imageView, nullptr);
    vmaDestroyImage(_deviceManager->allocator(), img.image, img.allocation);
}

GPUMeshBuffers ResourceManager::uploadMesh(std::span<uint32_t> indices, std::span<Vertex> vertices)
{
    const size_t vertexBufferSize = vertices.size() * sizeof(Vertex);
    const size_t indexBufferSize = indices.size() * sizeof(uint32_t);

    GPUMeshBuffers newSurface{};

    //create vertex buffer
    newSurface.vertexBuffer = create_buffer(vertexBufferSize,
                                            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                            VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                                            VMA_MEMORY_USAGE_GPU_ONLY);

    //find the adress of the vertex buffer
    VkBufferDeviceAddressInfo deviceAdressInfo{
        .sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO, .buffer = newSurface.vertexBuffer.buffer
    };
    newSurface.vertexBufferAddress = vkGetBufferDeviceAddress(_deviceManager->device(), &deviceAdressInfo);

    //create index buffer
    newSurface.indexBuffer = create_buffer(indexBufferSize,
                                           VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                           VMA_MEMORY_USAGE_GPU_ONLY);

    AllocatedBuffer staging = create_buffer(vertexBufferSize + indexBufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                            VMA_MEMORY_USAGE_CPU_ONLY);

    VmaAllocationInfo allocInfo{};
    vmaGetAllocationInfo(_deviceManager->allocator(), staging.allocation, &allocInfo);
    void *data = allocInfo.pMappedData;

    // copy vertex/index data to staging (host visible)
    memcpy(data, vertices.data(), vertexBufferSize);
    memcpy((char *) data + vertexBufferSize, indices.data(), indexBufferSize);
    // Ensure visibility on non-coherent memory before GPU copies
    vmaFlushAllocation(_deviceManager->allocator(), staging.allocation, 0, vertexBufferSize + indexBufferSize);

    PendingBufferUpload pending{};
    pending.staging = staging;
    pending.copies.push_back(BufferCopyRegion{
        .destination = newSurface.vertexBuffer.buffer,
        .dstOffset = 0,
        .size = vertexBufferSize,
        .stagingOffset = 0,
    });
    pending.copies.push_back(BufferCopyRegion{
        .destination = newSurface.indexBuffer.buffer,
        .dstOffset = 0,
        .size = indexBufferSize,
        .stagingOffset = vertexBufferSize,
    });

    _pendingBufferUploads.push_back(std::move(pending));

    if (!_deferUploads)
    {
        process_queued_uploads_immediate();
    }

    return newSurface;
}

bool ResourceManager::has_pending_uploads() const
{
    return !_pendingBufferUploads.empty() || !_pendingImageUploads.empty();
}

void ResourceManager::clear_pending_uploads()
{
    for (auto &upload : _pendingBufferUploads)
    {
        destroy_buffer(upload.staging);
    }
    for (auto &upload : _pendingImageUploads)
    {
        destroy_buffer(upload.staging);
    }
    _pendingBufferUploads.clear();
    _pendingImageUploads.clear();
}

void ResourceManager::process_queued_uploads_immediate()
{
    if (!has_pending_uploads()) return;

    immediate_submit([&](VkCommandBuffer cmd) {
        for (auto &bufferUpload : _pendingBufferUploads)
        {
            for (const auto &copy : bufferUpload.copies)
            {
                VkBufferCopy region{};
                region.srcOffset = copy.stagingOffset;
                region.dstOffset = copy.dstOffset;
                region.size = copy.size;
                vkCmdCopyBuffer(cmd, bufferUpload.staging.buffer, copy.destination, 1, &region);
            }
        }

        for (auto &imageUpload : _pendingImageUploads)
        {
            vkutil::transition_image(cmd, imageUpload.image, imageUpload.initialLayout,
                                     VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);

            VkBufferImageCopy copyRegion = {};
            copyRegion.bufferOffset = 0;
            copyRegion.bufferRowLength = 0;
            copyRegion.bufferImageHeight = 0;
            copyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            copyRegion.imageSubresource.mipLevel = 0;
            copyRegion.imageSubresource.baseArrayLayer = 0;
            copyRegion.imageSubresource.layerCount = 1;
            copyRegion.imageExtent = imageUpload.extent;

            vkCmdCopyBufferToImage(cmd,
                                   imageUpload.staging.buffer,
                                   imageUpload.image,
                                   VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                   1,
                                   &copyRegion);

            if (imageUpload.generateMips)
            {
                vkutil::generate_mipmaps(cmd, imageUpload.image,
                                         VkExtent2D{imageUpload.extent.width, imageUpload.extent.height});
            }
            else
            {
                vkutil::transition_image(cmd, imageUpload.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                         imageUpload.finalLayout);
            }
        }
    });

    clear_pending_uploads();
}

void ResourceManager::register_upload_pass(RenderGraph &graph, FrameResources &frame)
{
    if (_pendingBufferUploads.empty() && _pendingImageUploads.empty()) return;

    auto bufferUploads = std::make_shared<std::vector<PendingBufferUpload>>(std::move(_pendingBufferUploads));
    auto imageUploads = std::make_shared<std::vector<PendingImageUpload>>(std::move(_pendingImageUploads));

    struct BufferBinding
    {
        size_t uploadIndex{};
        RGBufferHandle stagingHandle{};
        std::vector<RGBufferHandle> destinationHandles;
    };

    struct ImageBinding
    {
        size_t uploadIndex{};
        RGBufferHandle stagingHandle{};
        RGImageHandle imageHandle{};
    };

    auto bufferBindings = std::make_shared<std::vector<BufferBinding>>();
    auto imageBindings = std::make_shared<std::vector<ImageBinding>>();

    bufferBindings->reserve(bufferUploads->size());
    imageBindings->reserve(imageUploads->size());

    std::unordered_map<VkBuffer, RGBufferHandle> destBufferHandles;
    std::unordered_map<VkImage, RGImageHandle> imageHandles;

    for (size_t i = 0; i < bufferUploads->size(); ++i)
    {
        const auto &upload = bufferUploads->at(i);
        BufferBinding binding{};
        binding.uploadIndex = i;

        RGImportedBufferDesc stagingDesc{};
        stagingDesc.name = std::string("upload.staging.buffer.") + std::to_string(i);
        stagingDesc.buffer = upload.staging.buffer;
        stagingDesc.size = upload.staging.info.size;
        stagingDesc.currentStage = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
        stagingDesc.currentAccess = 0;
        binding.stagingHandle = graph.import_buffer(stagingDesc);

        binding.destinationHandles.reserve(upload.copies.size());
        for (const auto &copy : upload.copies)
        {
            RGBufferHandle handle{};
            auto it = destBufferHandles.find(copy.destination);
            if (it == destBufferHandles.end())
            {
                RGImportedBufferDesc dstDesc{};
                dstDesc.name = std::string("upload.dst.buffer.") + std::to_string(destBufferHandles.size());
                dstDesc.buffer = copy.destination;
                dstDesc.size = copy.dstOffset + copy.size;
                dstDesc.currentStage = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
                dstDesc.currentAccess = 0;
                handle = graph.import_buffer(dstDesc);
                destBufferHandles.emplace(copy.destination, handle);
            }
            else
            {
                handle = it->second;
            }
            binding.destinationHandles.push_back(handle);
        }

        bufferBindings->push_back(std::move(binding));
    }

    for (size_t i = 0; i < imageUploads->size(); ++i)
    {
        const auto &upload = imageUploads->at(i);
        ImageBinding binding{};
        binding.uploadIndex = i;

        RGImportedBufferDesc stagingDesc{};
        stagingDesc.name = std::string("upload.staging.image.") + std::to_string(i);
        stagingDesc.buffer = upload.staging.buffer;
        stagingDesc.size = upload.staging.info.size;
        stagingDesc.currentStage = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
        stagingDesc.currentAccess = 0;
        binding.stagingHandle = graph.import_buffer(stagingDesc);

        auto it = imageHandles.find(upload.image);
        if (it == imageHandles.end())
        {
            RGImportedImageDesc imgDesc{};
            imgDesc.name = std::string("upload.image.") + std::to_string(imageHandles.size());
            imgDesc.image = upload.image;
            imgDesc.imageView = VK_NULL_HANDLE;
            imgDesc.format = upload.format;
            imgDesc.extent = {upload.extent.width, upload.extent.height};
            imgDesc.currentLayout = upload.initialLayout;
            binding.imageHandle = graph.import_image(imgDesc);
            imageHandles.emplace(upload.image, binding.imageHandle);
        }
        else
        {
            binding.imageHandle = it->second;
        }

        imageBindings->push_back(std::move(binding));
    }

    graph.add_pass("ResourceUploads", RGPassType::Transfer,
        [bufferBindings, imageBindings](RGPassBuilder &builder, EngineContext *)
        {
            for (const auto &binding : *bufferBindings)
            {
                builder.read_buffer(binding.stagingHandle, RGBufferUsage::TransferSrc);
                for (auto handle : binding.destinationHandles)
                {
                    builder.write_buffer(handle, RGBufferUsage::TransferDst);
                }
            }
            for (const auto &binding : *imageBindings)
            {
                builder.read_buffer(binding.stagingHandle, RGBufferUsage::TransferSrc);
                builder.write(binding.imageHandle, RGImageUsage::TransferDst);
            }
        },
        [bufferUploads, imageUploads, bufferBindings, imageBindings, this](VkCommandBuffer cmd, const RGPassResources &res, EngineContext *)
        {
            for (const auto &binding : *bufferBindings)
            {
                const auto &upload = bufferUploads->at(binding.uploadIndex);
                VkBuffer staging = res.buffer(binding.stagingHandle);
                for (size_t copyIndex = 0; copyIndex < upload.copies.size(); ++copyIndex)
                {
                    const auto &copy = upload.copies[copyIndex];
                    VkBuffer destination = res.buffer(binding.destinationHandles[copyIndex]);
                    VkBufferCopy region{};
                    region.srcOffset = copy.stagingOffset;
                    region.dstOffset = copy.dstOffset;
                    region.size = copy.size;
                    vkCmdCopyBuffer(cmd, staging, destination, 1, &region);
                }
            }

            for (const auto &binding : *imageBindings)
            {
                const auto &upload = imageUploads->at(binding.uploadIndex);
                VkBuffer staging = res.buffer(binding.stagingHandle);
                VkImage image = res.image(binding.imageHandle);

                VkBufferImageCopy region{};
                region.bufferOffset = 0;
                region.bufferRowLength = 0;
                region.bufferImageHeight = 0;
                region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                region.imageSubresource.mipLevel = 0;
                region.imageSubresource.baseArrayLayer = 0;
                region.imageSubresource.layerCount = 1;
                region.imageExtent = upload.extent;

                vkCmdCopyBufferToImage(cmd, staging, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

                if (upload.generateMips)
                {
                    vkutil::generate_mipmaps(cmd, image, VkExtent2D{upload.extent.width, upload.extent.height});
                    vkutil::transition_image(cmd, image, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
                }
            }
        });

    frame._deletionQueue.push_function([buffers = bufferUploads, images = imageUploads, this]()
    {
        for (const auto &upload : *buffers)
        {
            destroy_buffer(upload.staging);
        }
        for (const auto &upload : *images)
        {
            destroy_buffer(upload.staging);
        }
    });
}
