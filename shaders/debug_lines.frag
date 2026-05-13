#version 450

layout(location = 0) in vec4 inColor;
layout(location = 1) noperspective in float inSide;
layout(location = 2) flat in float inEdgeSoftness;
layout(location = 3) noperspective in float inDashCoordPx;
layout(location = 0) out vec4 outColor;

void main()
{
    outColor = inColor;
    const float edgeDist = abs(inSide);
    const float coverage = 1.0 - smoothstep(inEdgeSoftness, 1.0, edgeDist);
    float dashCoverage = 1.0;
    if (inDashCoordPx >= 0.0)
    {
        const float dashOnPx = 14.0;
        const float dashOffPx = 9.0;
        const float dashEdgePx = 1.0;
        const float phasePx = mod(inDashCoordPx, dashOnPx + dashOffPx);
        dashCoverage = 1.0 - smoothstep(dashOnPx - dashEdgePx, dashOnPx, phasePx);
    }
    outColor.a *= coverage * dashCoverage;
    if (outColor.a <= 0.0)
    {
        discard;
    }
}
