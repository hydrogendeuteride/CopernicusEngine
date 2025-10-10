#pragma once

#include <memory>
#include <core/vk_types.h>
#include <core/vk_descriptors.h>
// Avoid including vk_scene.h here to prevent cycles

struct EngineStats
{
    float frametime;
    int triangle_count;
    int drawcall_count;
    float scene_update_time;
    float mesh_draw_time;
};

class DeviceManager;
class ResourceManager;
class SwapchainManager;
class DescriptorManager;
class SamplerManager;
class SceneManager;
class MeshAsset;
struct DrawContext;
struct GPUSceneData;
class ComputeManager;
class PipelineManager;
struct FrameResources;
struct SDL_Window;
class AssetManager;
class RenderGraph;

class EngineContext
{
public:
    // Owned shared resources
    std::shared_ptr<DeviceManager> device;
    std::shared_ptr<ResourceManager> resources;
    std::shared_ptr<DescriptorAllocatorGrowable> descriptors;

    // Non-owning pointers to global managers owned by VulkanEngine
    SwapchainManager* swapchain = nullptr;
    DescriptorManager* descriptorLayouts = nullptr;
    SamplerManager* samplers = nullptr;
    SceneManager* scene = nullptr;

    // Per-frame and subsystem pointers for modules to use without VulkanEngine
    FrameResources* currentFrame = nullptr;      // set by engine each frame
    EngineStats* stats = nullptr;                // points to engine stats
    ComputeManager* compute = nullptr;           // compute subsystem
    PipelineManager* pipelines = nullptr;        // graphics pipeline manager
    RenderGraph* renderGraph = nullptr;          // render graph (built per-frame)
    SDL_Window* window = nullptr;                // SDL window handle

    // Frequently used values
    VkExtent2D drawExtent{};

    // Optional convenience content pointers (moved to AssetManager for meshes)

    // Assets
    AssetManager* assets = nullptr;              // non-owning pointer to central AssetManager

    // Accessors
    DeviceManager *getDevice() const { return device.get(); }
    ResourceManager *getResources() const { return resources.get(); }
    DescriptorAllocatorGrowable *getDescriptors() const { return descriptors.get(); }
    SwapchainManager* getSwapchain() const { return swapchain; }
    DescriptorManager* getDescriptorLayouts() const { return descriptorLayouts; }
    SamplerManager* getSamplers() const { return samplers; }
    const GPUSceneData& getSceneData() const;
    const DrawContext& getMainDrawContext() const;
    VkExtent2D getDrawExtent() const { return drawExtent; }
    AssetManager* getAssets() const { return assets; }
    // Convenience alias (singular) requested
    AssetManager* getAsset() const { return assets; }
    RenderGraph* getRenderGraph() const { return renderGraph; }
};
