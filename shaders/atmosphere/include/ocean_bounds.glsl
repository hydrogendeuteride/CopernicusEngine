#ifndef ATMOSPHERE_OCEAN_BOUNDS_GLSL
#define ATMOSPHERE_OCEAN_BOUNDS_GLSL

#include "planet_gbuffer_payload.glsl"

const float PLANET_OCEAN_MASK_THRESHOLD = 0.5;

bool gbuffer_pixel_is_ocean(float posW)
{
    return decode_planet_gbuffer_is_planet(posW) &&
           decode_planet_gbuffer_water_mask(posW) >= PLANET_OCEAN_MASK_THRESHOLD;
}

bool solve_ocean_depth(vec3 camLocal,
                       vec3 rd,
                       vec3 center,
                       float planetRadius,
                       out float tSea)
{
    float seaRadius = planetRadius + max(pc.terrain_params.y, 0.0) + max(pc.terrain_params.z, 0.0);
    float t0;
    float t1;
    if (!ray_sphere_intersect(camLocal, rd, center, seaRadius, t0, t1))
    {
        return false;
    }

    tSea = (t0 > 0.0) ? t0 : t1;
    return tSea > 0.0;
}

#endif
