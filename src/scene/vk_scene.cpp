#include "vk_scene.h"

#include <utility>

#include "vk_swapchain.h"
#include "core/engine_context.h"
#include "core/config.h"
#include "glm/gtx/transform.hpp"
#include <glm/gtc/matrix_transform.hpp>

#include "glm/gtx/norm.inl"

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

    // Mixed Near + CSM shadow setup
    // - Cascade 0: legacy/simple shadow (near range around camera)
    // - Cascades 1..N-1: cascaded shadow maps covering farther ranges up to kShadowCSMFar

    // ---- Mixed Near + CSM shadow setup (fixed) ----
    {
        const glm::mat4 invView = glm::inverse(view);
        const glm::vec3 camPos = glm::vec3(invView[3]);

        glm::vec3 L = glm::normalize(-glm::vec3(sceneData.sunlightDirection));
        if (glm::length(L) < 1e-5f) L = glm::vec3(0.0f, -1.0f, 0.0f);
        const glm::vec3 worldUp(0.0f, 1.0f, 0.0f);
        glm::vec3 right = glm::cross(L, worldUp);
        if (glm::length2(right) < 1e-6f) right = glm::vec3(1, 0, 0);
        right = glm::normalize(right);
        glm::vec3 up = glm::normalize(glm::cross(right, L)); {
            const float orthoRange = 10.0f;
            const float nearDist = 0.1f;
            const float farDist = 200.0f;
            const glm::vec3 lightPos = camPos - L * 50.0f;
            const glm::mat4 viewLight = glm::lookAtRH(lightPos, camPos, up);

            const glm::mat4 projLight = glm::orthoRH_ZO(-orthoRange, orthoRange, -orthoRange, orthoRange,
                                                        nearDist, farDist);

            const glm::mat4 lightVP = projLight * viewLight;
            sceneData.lightViewProj = lightVP;
            sceneData.lightViewProjCascades[0] = lightVP;
        }

        const float farView = kShadowCSMFar;
        const float nearSplit = 5.0f;
        const float lambda = 1.0f;
        float splits[3]{};
        for (int i = 1; i <= 3; ++i)
        {
            float si = float(i) / 3.0f;
            float logd = nearSplit * powf(farView / nearSplit, si);
            float lind = glm::mix(nearSplit, farView, si);
            splits[i - 1] = glm::mix(lind, logd, lambda);
        }
        sceneData.cascadeSplitsView = glm::vec4(nearSplit, splits[0], splits[1], farView);

        auto frustum_corners_world = [&](float zn, float zf) {
            const float tanHalfFov = tanf(fov * 0.5f);
            const float yN = tanHalfFov * zn;
            const float xN = yN * aspect;
            const float yF = tanHalfFov * zf;
            const float xF = yF * aspect;

            glm::vec3 vs[8] = {
                {-xN, -yN, -zn}, {+xN, -yN, -zn}, {+xN, +yN, -zn}, {-xN, +yN, -zn},
                {-xF, -yF, -zf}, {+xF, -yF, -zf}, {+xF, +yF, -zf}, {-xF, +yF, -zf}
            };
            std::array<glm::vec3, 8> ws{};
            for (int i = 0; i < 8; ++i)
                ws[i] = glm::vec3(invView * glm::vec4(vs[i], 1.0f));
            return ws;
        };

        auto build_light_matrix_for_slice = [&](float zNearVS, float zFarVS) {
            auto ws = frustum_corners_world(zNearVS, zFarVS);

            glm::vec3 center(0.0f);
            for (auto &p: ws) center += p;
            center *= (1.0f / 8.0f);
            const float lightPullback = 20.0f;
            glm::mat4 V = glm::lookAtRH(center - L * lightPullback, center, up);

            glm::vec3 minLS(FLT_MAX), maxLS(-FLT_MAX);
            for (auto &p: ws)
            {
                glm::vec3 q = glm::vec3(V * glm::vec4(p, 1.0f));
                minLS = glm::min(minLS, q);
                maxLS = glm::max(maxLS, q);
            }

            glm::vec2 extXY = glm::vec2(maxLS.x - minLS.x, maxLS.y - minLS.y);
            float radius = 0.5f * glm::max(extXY.x, extXY.y);
            radius = radius * kShadowCascadeRadiusScale + kShadowCascadeRadiusMargin;

            glm::vec2 centerXY = 0.5f * glm::vec2(maxLS.x + minLS.x, maxLS.y + minLS.y);

            const float texel = (2.0f * radius) / float(kShadowMapResolution);
            centerXY.x = floorf(centerXY.x / texel) * texel;
            centerXY.y = floorf(centerXY.y / texel) * texel;

            glm::mat4 Vsnapped = glm::translate(glm::mat4(1.0f),
                                                -glm::vec3(centerXY.x, centerXY.y, 0.0f)) * V;

            const float zPad = 50.0f;
            float zNear = glm::max(0.1f, -maxLS.z - zPad);
            float zFar = -minLS.z + zPad;

            glm::mat4 P = glm::orthoRH_ZO(-radius, radius, -radius, radius, zNear, zFar);

            return P * Vsnapped;
        };

        float prev = nearSplit;
        for (int ci = 1; ci < kShadowCascadeCount; ++ci)
        {
            float end = (ci < kShadowCascadeCount - 1) ? splits[ci - 1] : farView;
            sceneData.lightViewProjCascades[ci] = build_light_matrix_for_slice(prev, end);
            prev = end;
        }
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
