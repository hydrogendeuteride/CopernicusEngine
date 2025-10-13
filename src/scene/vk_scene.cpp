#include "vk_scene.h"

#include <utility>

#include "vk_swapchain.h"
#include "core/engine_context.h"
#include "glm/gtx/transform.hpp"
#include <glm/gtc/matrix_transform.hpp>

#include "glm/gtx/norm.inl"
#include "glm/gtx/compatibility.hpp"
#include <algorithm>
#include "core/config.h"

void SceneManager::init(EngineContext *context)
{
    _context = context;

    mainCamera.velocity = glm::vec3(0.f);
    mainCamera.position = glm::vec3(30.f, -00.f, 85.f);
    mainCamera.pitch = 0;
    mainCamera.yaw = 0;

    sceneData.ambientColor = glm::vec4(0.1f, 0.1f, 0.1f, 1.0f);
    sceneData.sunlightDirection = glm::vec4(-1.0f, -1.0f, -1.0f, 1.0f);
    sceneData.sunlightColor = glm::vec4(1.0f, 1.0f, 1.0f, 3.0f);
}

void SceneManager::update_scene()
{
    auto start = std::chrono::system_clock::now();

    mainDrawContext.OpaqueSurfaces.clear();
    mainDrawContext.TransparentSurfaces.clear();

    mainCamera.update();

    if (loadedScenes.find("structure") != loadedScenes.end())
    {
        loadedScenes["structure"]->Draw(glm::mat4{1.f}, mainDrawContext);
    }

    // dynamic GLTF instances
    for (const auto &kv: dynamicGLTFInstances)
    {
        const GLTFInstance &inst = kv.second;
        if (inst.scene)
        {
            inst.scene->Draw(inst.transform, mainDrawContext);
        }
    }

    // Default primitives are added as dynamic instances by the engine.

    // dynamic mesh instances
    for (const auto &kv: dynamicMeshInstances)
    {
        const MeshInstance &inst = kv.second;
        if (!inst.mesh || inst.mesh->surfaces.empty()) continue;
        for (const auto &surf: inst.mesh->surfaces)
        {
            RenderObject obj{};
            obj.indexCount = surf.count;
            obj.firstIndex = surf.startIndex;
            obj.indexBuffer = inst.mesh->meshBuffers.indexBuffer.buffer;
            obj.vertexBuffer = inst.mesh->meshBuffers.vertexBuffer.buffer;
            obj.vertexBufferAddress = inst.mesh->meshBuffers.vertexBufferAddress;
            obj.material = &surf.material->data;
            obj.bounds = surf.bounds;
            obj.transform = inst.transform;
            if (obj.material->passType == MaterialPass::Transparent)
            {
                mainDrawContext.TransparentSurfaces.push_back(obj);
            }
            else
            {
                mainDrawContext.OpaqueSurfaces.push_back(obj);
            }
        }
    }

    glm::mat4 view = mainCamera.getViewMatrix();
    // Use reversed infinite-Z projection (right-handed, -Z forward) to avoid far-plane clipping
    // on very large scenes. Vulkan clip space is 0..1 (GLM_FORCE_DEPTH_ZERO_TO_ONE) and requires Y flip.
    auto makeReversedInfinitePerspective = [](float fovyRadians, float aspect, float zNear) {
        // Column-major matrix; indices are [column][row]
        float f = 1.0f / tanf(fovyRadians * 0.5f);
        glm::mat4 m(0.0f);
        m[0][0] = f / aspect;
        m[1][1] = f;
        m[2][2] = 0.0f;
        m[2][3] = -1.0f; // w = -z_eye (right-handed)
        m[3][2] = zNear; // maps near -> 1, far -> 0 (reversed-Z)
        return m;
    };

    const float fov = glm::radians(70.f);
    const float aspect = (float) _context->getSwapchain()->windowExtent().width /
                         (float) _context->getSwapchain()->windowExtent().height;
    const float nearPlane = 0.1f;
    glm::mat4 projection = makeReversedInfinitePerspective(fov, aspect, nearPlane);
    // Vulkan NDC has inverted Y.
    projection[1][1] *= -1.0f;

    sceneData.view = view;
    sceneData.proj = projection;
    sceneData.viewproj = projection * view;

    // Build cascaded directional light view-projection matrices
    {
        using namespace glm;
        const vec3 camPos = vec3(inverse(view)[3]);
        // Use light-ray direction (from light to scene).
        // Shaders compute per-fragment L as -sunlightDirection (vector to light).
        vec3 L = normalize(vec3(sceneData.sunlightDirection));
        if (!glm::all(glm::isfinite(L)) || glm::length2(L) < 1e-10f)
            L = glm::vec3(0.0f, -1.0f, 0.0f);

        const glm::vec3 worldUp(0,1,0), altUp(0,0,1);
        glm::vec3 upPick = (std::abs(glm::dot(worldUp, L)) > 0.99f) ? altUp : worldUp;
        glm::vec3 right = glm::normalize(glm::cross(upPick, L));
        glm::vec3 up    = glm::normalize(glm::cross(L, right));

        const float csmFar = kShadowCSMFar; // configurable shadow distance
        const float lambda = 0.8f; // split weighting
        const int cascades = kShadowCascadeCount;

        float splits[4] = {0, 0, 0, 0};
        for (int i = 1; i <= cascades; ++i)
        {
            float p = (float) i / (float) cascades;
            float logd = nearPlane * std::pow(csmFar / nearPlane, p);
            float lind = nearPlane + (csmFar - nearPlane) * p;
            float d = glm::mix(lind, logd, lambda);
            if (i - 1 < 4) splits[i - 1] = d;
        }
        sceneData.cascadeSplitsView = vec4(splits[0], splits[1], splits[2], splits[3]);

        mat4 invView = inverse(view);

        auto buildCascade = [&](float nearD, float farD) -> mat4 {
            // Frustum in view-space (RH, forward -Z)
            float tanHalfFov = tanf(fov * 0.5f);
            float yn = tanHalfFov * nearD;
            float xn = yn * aspect;
            float yf = tanHalfFov * farD;
            float xf = yf * aspect;

            vec3 cornersV[8] = {
                {-xn, -yn, -nearD}, {xn, -yn, -nearD}, {xn, yn, -nearD}, {-xn, yn, -nearD},
                {-xf, -yf, -farD}, {xf, -yf, -farD}, {xf, yf, -farD}, {-xf, yf, -farD}
            };
            vec3 cornersW[8];
            vec3 centerWS(0.0f);
            for (int i = 0; i < 8; ++i)
            {
                vec3 w = vec3(invView * vec4(cornersV[i], 1.0f));
                cornersW[i] = w;
                centerWS += w;
            }
            centerWS *= (1.0f / 8.0f);

            // Initial light view
            const float lightDist = 100.0f;
            vec3 lightPos = centerWS - L * lightDist;
            mat4 viewLight = lookAtRH(lightPos, centerWS, up);

            // Compute symmetric bounds around center in light space
            vec2 centerLS = vec2(viewLight * vec4(centerWS, 1.0f));
            float minZ = 1e9f, maxZ = -1e9f;
            float radius = 0.0f;
            for (int i = 0; i < 8; ++i)
            {
                vec3 p = vec3(viewLight * vec4(cornersW[i], 1.0f));
                minZ = std::min(minZ, p.z);
                maxZ = std::max(maxZ, p.z);
                radius = std::max(radius, glm::length(vec2(p.x, p.y) - centerLS));
            }

            // Pad extents
            radius *= 1.05f;
            float sliceLen = farD - nearD;
            float zPad = std::max(50.0f, 0.2f * sliceLen);
            // Two-sided along light direction: include casters between light and slice
            float nearLS = 0.01f;
            float farLS  = -minZ + zPad;

            // Stabilize by snapping to shadow texel grid
            float texelSize = (2.0f * radius) / kShadowMapResolution;
            vec2 snapped = floor(centerLS / texelSize) * texelSize;
            vec2 deltaLS = snapped - centerLS;
            vec3 shiftWS = right * deltaLS.x + up * deltaLS.y;
            vec3 centerSnapped = centerWS + shiftWS;
            vec3 lightPosSnapped = centerSnapped - L * lightDist;
            viewLight = lookAtRH(lightPosSnapped, centerSnapped, up);

            // Recompute z-range with snapped view
            centerLS = vec2(viewLight * vec4(centerSnapped, 1.0f));
            minZ = 1e9f; maxZ = -1e9f; radius = 0.0f;
            for (int i = 0; i < 8; ++i)
            {
                vec3 p = vec3(viewLight * vec4(cornersW[i], 1.0f));
                minZ = std::min(minZ, p.z);
                maxZ = std::max(maxZ, p.z);
                radius = std::max(radius, glm::length(vec2(p.x, p.y) - centerLS));
            }
            // Keep near plane close to the light to include forward casters
            nearLS = 0.01f;
            farLS  = -minZ + zPad;

            float left   = centerLS.x - radius;
            float rightE = centerLS.x + radius;
            float bottom = centerLS.y - radius;
            float top    = centerLS.y + radius;

            mat4 projLight = orthoRH_ZO(left, rightE, bottom, top, nearLS, farLS);
            return projLight * viewLight;
        };

        for (int i = 0; i < cascades; ++i)
        {
            float nearD = (i == 0) ? nearPlane : splits[i - 1];
            float farD = splits[i];
            sceneData.lightViewProjCascades[i] = buildCascade(nearD, farD);
        }
        // For legacy paths, keep first cascade in single matrix
        sceneData.lightViewProj = sceneData.lightViewProjCascades[0];
    }

    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    stats.scene_update_time = elapsed.count() / 1000.f;
}

void SceneManager::loadScene(const std::string &name, std::shared_ptr<LoadedGLTF> scene)
{
    loadedScenes[name] = std::move(scene);
}

std::shared_ptr<LoadedGLTF> SceneManager::getScene(const std::string &name)
{
    auto it = loadedScenes.find(name);
    return (it != loadedScenes.end()) ? it->second : nullptr;
}

void SceneManager::cleanup()
{
    loadedScenes.clear();
    loadedNodes.clear();
}

void SceneManager::addMeshInstance(const std::string &name, std::shared_ptr<MeshAsset> mesh, const glm::mat4 &transform)
{
    if (!mesh) return;
    dynamicMeshInstances[name] = MeshInstance{std::move(mesh), transform};
}

bool SceneManager::removeMeshInstance(const std::string &name)
{
    return dynamicMeshInstances.erase(name) > 0;
}

void SceneManager::clearMeshInstances()
{
    dynamicMeshInstances.clear();
}

void SceneManager::addGLTFInstance(const std::string &name, std::shared_ptr<LoadedGLTF> scene,
                                   const glm::mat4 &transform)
{
    if (!scene) return;
    dynamicGLTFInstances[name] = GLTFInstance{std::move(scene), transform};
}

bool SceneManager::removeGLTFInstance(const std::string &name)
{
    return dynamicGLTFInstances.erase(name) > 0;
}

void SceneManager::clearGLTFInstances()
{
    dynamicGLTFInstances.clear();
}
