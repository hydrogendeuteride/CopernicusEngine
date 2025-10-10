#version 450

layout(location=0) in vec2 inUV;
layout(location=0) out vec4 outColor;

layout(set=0, binding=0) uniform sampler2D uHdr;

layout(push_constant) uniform Push
{
    float exposure;
    int   mode;
} pc;

vec3 reinhard(vec3 x)
{
    return x / (1.0 + x);
}

// Narkowicz ACES approximation
vec3 aces_tonemap(vec3 x)
{
    // https://64.github.io/tonemapping/
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((x*(a*x+b))/(x*(c*x+d)+e), 0.0, 1.0);
}

void main()
{
    vec3 hdr = texture(uHdr, inUV).rgb;

    // Simple exposure
    float exposure = max(pc.exposure, 0.0001);
    vec3 mapped = hdr * exposure;

    if (pc.mode == 1)
        mapped = aces_tonemap(mapped);
    else
        mapped = reinhard(mapped);

    const float gamma = 2.2;
    mapped = pow(mapped, vec3(1.0 / gamma));

    outColor = vec4(mapped, 1.0);
}

