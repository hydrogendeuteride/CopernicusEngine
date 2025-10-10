## Compute System: Pipelines, Instances, and Dispatch

Standalone compute subsystem with a small, explicit API. Used by passes (e.g., Background) and tools. It lives under `src/compute` and is surfaced via `EngineContext::compute` and convenience wrappers on `PipelineManager`.

### Concepts

- Pipelines: Named compute pipelines created from a SPIR‑V module and a simple descriptor layout spec.
- Instances: Persistently bound descriptor sets keyed by instance name; useful for effects that rebind images/buffers across frames without re‑creating pipelines.
- Dispatch: Issue work with group counts, optional push constants, and ad‑hoc memory barriers.

### Key Types

- `ComputePipelineCreateInfo` — shader path, descriptor types, push constant size/stages, optional specialization (src/compute/vk_compute.h).
- `ComputeDispatchInfo` — `groupCount{X,Y,Z}`, `bindings`, `pushConstants`, and `*_barriers` arrays for additional sync.
- `ComputeBinding` — helpers for `uniformBuffer`, `storageBuffer`, `sampledImage`, `storeImage`.

### API Surface

- Register/Destroy
  - `bool ComputeManager::registerPipeline(name, ComputePipelineCreateInfo)`
  - `void ComputeManager::unregisterPipeline(name)`
  - Query: `bool ComputeManager::hasPipeline(name)`

- Dispatch
  - `void ComputeManager::dispatch(cmd, name, ComputeDispatchInfo)`
  - `void ComputeManager::dispatchImmediate(name, ComputeDispatchInfo)` — records on a transient command buffer and submits.
  - Helpers: `createDispatch2D(w,h[,lsX,lsY])`, `createDispatch3D(w,h,d[,lsX,lsY,lsZ])`.

- Instances
  - `bool ComputeManager::createInstance(instanceName, pipelineName)` / `destroyInstance(instanceName)`
  - `setInstanceStorageImage`, `setInstanceSampledImage`, `setInstanceBuffer`
  - `AllocatedImage createAndBindStorageImage(...)`, `AllocatedBuffer createAndBindStorageBuffer(...)`
  - `void dispatchInstance(cmd, instanceName, info)`

### Quick Start — One‑Shot Dispatch

```c++
ComputePipelineCreateInfo ci{};
ci.shaderPath = context->getAssets()->shaderPath("blur.comp.spv");
ci.descriptorTypes = { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE };
ci.pushConstantSize = sizeof(ComputePushConstants);
context->compute->registerPipeline("blur", ci);

ComputeDispatchInfo di = ComputeManager::createDispatch2D(draw.w, draw.h);
di.bindings.push_back(ComputeBinding::storeImage(0, outImageView));
di.bindings.push_back(ComputeBinding::sampledImage(1, inImageView, context->getSamplers()->defaultLinear()));
ComputePushConstants pc{}; /* fill */
di.pushConstants = &pc; di.pushConstantSize = sizeof(pc);
context->compute->dispatch(cmd, "blur", di);
```

### Quick Start — Persistent Instance

```c++
context->compute->createInstance("background.sky", "sky");
context->compute->setInstanceStorageImage("background.sky", 0, ctx->getSwapchain()->drawImage().imageView);

ComputeDispatchInfo di = ComputeManager::createDispatch2D(ctx->getDrawExtent().width,
                                                          ctx->getDrawExtent().height);
di.pushConstants = &effect.data; di.pushConstantSize = sizeof(ComputePushConstants);
context->compute->dispatchInstance(cmd, "background.sky", di);
```

### Integration With Render Graph

- Compute passes declare `write(image, RGImageUsage::ComputeWrite)` in their build callback; the graph inserts layout transitions to `GENERAL` and required barriers.
- Background pass example: `src/render/vk_renderpass_background.cpp`.

### Sync Notes

- ComputeManager inserts minimal barriers needed for common cases; prefer using the Render Graph for cross‑pass synchronization.
- For advanced cases, add `imageBarriers`/`bufferBarriers` to `ComputeDispatchInfo`.

