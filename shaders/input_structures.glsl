// Maximum number of shadow cascades supported in shaders
#define MAX_CASCADES 4

layout(set = 0, binding = 0) uniform  SceneData{

    mat4 view;
    mat4 proj;
    mat4 viewproj;
    mat4 lightViewProj; // legacy single-shadow for fallback
    vec4 ambientColor;
    vec4 sunlightDirection; //w for sun power
    vec4 sunlightColor;
    // CSM data
    mat4 lightViewProjCascades[MAX_CASCADES];
    vec4 cascadeSplitsView; // positive view-space distances of far plane per cascade
} sceneData;

layout(set = 1, binding = 0) uniform GLTFMaterialData{

    vec4 colorFactors;
    vec4 metal_rough_factors;

} materialData;

layout(set = 1, binding = 1) uniform sampler2D colorTex;
layout(set = 1, binding = 2) uniform sampler2D metalRoughTex;
