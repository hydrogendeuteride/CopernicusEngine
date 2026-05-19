#version 460
#extension GL_GOOGLE_include_directive : require

#include "input_structures.glsl"

layout(location = 0) in vec2 inUV;
layout(location = 1) in vec3 inWorldRay;
layout(location = 2) flat in vec3 inCamLocal;

layout(location = 0) out vec4 outCloudLighting;
layout(location = 1) out vec4 outCloudSegment;

layout(set = 1, binding = 0) uniform sampler2D posTex;
layout(set = 1, binding = 1) uniform sampler2D cloudLightingLowResTex;
layout(set = 1, binding = 2) uniform sampler2D cloudSegmentLowResTex;
layout(set = 1, binding = 3) uniform sampler2D planetHeightTexPX;
layout(set = 1, binding = 4) uniform sampler2D planetHeightTexNX;
layout(set = 1, binding = 5) uniform sampler2D planetHeightTexPY;
layout(set = 1, binding = 6) uniform sampler2D planetHeightTexNY;
layout(set = 1, binding = 7) uniform sampler2D planetHeightTexPZ;
layout(set = 1, binding = 8) uniform sampler2D planetHeightTexNZ;

layout(push_constant) uniform AtmospherePush
{
    vec4 planet_center_radius;
    vec4 atmosphere_params;
    vec4 beta_rayleigh;
    vec4 beta_mie;
    vec4 jitter_params;
    vec4 terrain_params;
    vec4 cloud_layer;
    vec4 cloud_params;
    vec4 cloud_color;
    ivec4 misc;
} pc;

#include "atmosphere/include/constants.glsl"
#include "atmosphere/include/planet_heightmap.glsl"
#include "atmosphere/include/ray_bounds.glsl"

bool segment_valid(vec2 seg)
{
    return seg.x < seg.y;
}

int segment_count(vec4 segments)
{
    int count = 0;
    if (segment_valid(segments.xy)) count++;
    if (segment_valid(segments.zw)) count++;
    return count;
}

float segment_total_length(vec4 segments)
{
    float total = 0.0;
    if (segment_valid(segments.xy)) total += max(segments.y - segments.x, 0.0);
    if (segment_valid(segments.zw)) total += max(segments.w - segments.z, 0.0);
    return total;
}

float segment_error(vec4 lhs, vec4 rhs)
{
    int lhsCount = segment_count(lhs);
    int rhsCount = segment_count(rhs);
    if (lhsCount != rhsCount) return 1e20;

    float error = 0.0;
    if (lhsCount >= 1)
    {
        error += abs(lhs.x - rhs.x) + abs(lhs.y - rhs.y);
    }
    if (lhsCount == 2)
    {
        error += abs(lhs.z - rhs.z) + abs(lhs.w - rhs.w);
    }
    return error;
}

void main()
{
    outCloudLighting = vec4(0.0, 0.0, 0.0, 1.0);
    outCloudSegment = vec4(0.0);

    float planetRadius = pc.planet_center_radius.w;
    if (planetRadius <= 0.0) return;

    uint miscPacked = uint(pc.misc.w);
    int flags = int(miscPacked & MISC_FLAGS_MASK);
    bool wantClouds = (flags & FLAG_CLOUDS) != 0;
    if (!wantClouds) return;

    float cloudBaseM = max(pc.cloud_layer.x, 0.0);
    float cloudThicknessM = max(pc.cloud_layer.y, 0.0);
    float cloudDensScale = max(pc.cloud_layer.z, 0.0);
    float rBase = planetRadius + cloudBaseM;
    float rTop = rBase + cloudThicknessM;
    if (cloudThicknessM <= 0.0 || cloudDensScale <= 0.0 || rTop <= planetRadius) return;

    bool wantAtmosphere = (flags & FLAG_ATMOSPHERE) != 0;
    float atmRadius = pc.atmosphere_params.x;
    float boundRadius = (wantAtmosphere && atmRadius > planetRadius) ? atmRadius : rTop;

    vec3 camLocal = inCamLocal;
    vec3 rd = normalize(inWorldRay);
    vec3 center = pc.planet_center_radius.xyz;

    float tStart;
    float tEnd;
    if (!compute_primary_ray_bounds_cloud_hybrid(camLocal, rd, center, planetRadius, boundRadius, tStart, tEnd))
    {
        return;
    }

    vec2 analyticSeg0;
    vec2 analyticSeg1;
    int analyticCount = compute_cloud_segments(camLocal, rd, center, rBase, rTop, tStart, tEnd, analyticSeg0, analyticSeg1);
    vec4 analyticSegments = vec4(analyticSeg0, (analyticCount == 2 && segment_valid(analyticSeg1)) ? analyticSeg1 : vec2(0.0));
    if (segment_count(analyticSegments) == 0)
    {
        return;
    }

    ivec2 lowSize = textureSize(cloudSegmentLowResTex, 0);
    ivec2 baseCoord = clamp(ivec2(gl_FragCoord.xy) / 2, ivec2(0), lowSize - ivec2(1));

    float analyticLen = max(segment_total_length(analyticSegments), 1e-3);
    float countScale = float(max(analyticCount, 1));
    float rejectThreshold = max(3200.0, analyticLen * 0.60 * countScale);
    float blendThreshold = max(1200.0, analyticLen * 0.20 * countScale);
    vec4 accumulatedLighting = vec4(0.0);
    float accumulatedWeight = 0.0;
    bool found = false;
    vec4 fallbackLighting = vec4(0.0);
    float fallbackScore = 1e20;
    float fallbackDistance2 = 1e20;
    bool haveFallback = false;

    for (int y = -1; y <= 1; ++y)
    {
        for (int x = -1; x <= 1; ++x)
        {
            ivec2 coord = clamp(baseCoord + ivec2(x, y), ivec2(0), lowSize - ivec2(1));
            vec4 candidateSeg = texelFetch(cloudSegmentLowResTex, coord, 0);
            if (segment_count(candidateSeg) == 0)
            {
                continue;
            }

            float score = segment_error(candidateSeg, analyticSegments);
            float distance2 = float(x * x + y * y);
            if (!haveFallback ||
                score < fallbackScore ||
                (score == fallbackScore && distance2 < fallbackDistance2))
            {
                fallbackLighting = texelFetch(cloudLightingLowResTex, coord, 0);
                fallbackScore = score;
                fallbackDistance2 = distance2;
                haveFallback = true;
            }

            if (score > rejectThreshold)
            {
                continue;
            }

            float scoreNorm = score / max(blendThreshold, 1e-3);
            float segmentWeight = exp(-scoreNorm * scoreNorm);
            float spatialWeight = 1.0 / (1.0 + distance2);
            float weight = segmentWeight * spatialWeight;
            if (weight <= 1e-4)
            {
                continue;
            }

            accumulatedLighting += texelFetch(cloudLightingLowResTex, coord, 0) * weight;
            accumulatedWeight += weight;
            found = true;
        }
    }

    if (found && accumulatedWeight > 1e-4)
    {
        outCloudLighting = accumulatedLighting / accumulatedWeight;
        outCloudSegment = analyticSegments;
        return;
    }

    if (!haveFallback)
    {
        return;
    }

    outCloudLighting = fallbackLighting;
    outCloudSegment = analyticSegments;
}
