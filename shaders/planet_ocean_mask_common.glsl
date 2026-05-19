#ifndef PLANET_OCEAN_MASK_COMMON_GLSL
#define PLANET_OCEAN_MASK_COMMON_GLSL

const float PLANET_OCEAN_MASK_LO = 0.45;
const float PLANET_OCEAN_MASK_HI = 0.55;
const float PLANET_OCEAN_EDGE_MIN = 0.002;
const float PLANET_OCEAN_COVERAGE_EPS = 0.001;

float planet_ocean_strength()
{
    if (materialData.extra[2].z <= 0.5)
    {
        return 0.0;
    }

    return clamp(materialData.extra[2].w, 0.0, 1.0);
}

float sample_planet_ocean_mask(vec2 uv)
{
    float strength = planet_ocean_strength();
    if (strength <= 0.0)
    {
        return 0.0;
    }

    return clamp(textureLod(planetSpecularTex, uv, 0.0).r * strength, 0.0, 1.0);
}

float sample_planet_ocean_coverage(vec2 uv)
{
    float mask = sample_planet_ocean_mask(uv);
    float edge = max(fwidth(mask), PLANET_OCEAN_EDGE_MIN);
    return smoothstep(PLANET_OCEAN_MASK_LO - edge, PLANET_OCEAN_MASK_HI + edge, mask);
}

float planet_ocean_flag(float coverage)
{
    return coverage > PLANET_OCEAN_COVERAGE_EPS ? 1.0 : 0.0;
}

bool planet_ocean_visible(float coverage)
{
    return coverage > PLANET_OCEAN_COVERAGE_EPS;
}

#endif
