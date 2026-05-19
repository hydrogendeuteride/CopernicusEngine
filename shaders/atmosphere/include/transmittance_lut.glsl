const float TRANSMITTANCE_MAX_AIRMASS = 65504.0;

float transmittance_horizon_mu(float r, float planetRadius)
{
    if (planetRadius <= 0.0 || r <= 0.0)
    {
        return -1.0;
    }

    float q = clamp(planetRadius / r, 0.0, 1.0);
    return -sqrt(max(1.0 - q * q, 0.0));
}

bool transmittance_ray_hits_planet(float r, float mu, float planetRadius)
{
    if (planetRadius <= 0.0 || r <= 0.0)
    {
        return false;
    }

    if (r <= planetRadius)
    {
        return mu < 0.0;
    }

    return mu < transmittance_horizon_mu(r, planetRadius);
}

vec2 sample_transmittance_air_mass(sampler2D lut,
                                   float r,
                                   float mu,
                                   float planetRadius,
                                   float atmRadius)
{
    if (planetRadius <= 0.0 || atmRadius <= planetRadius || r <= 0.0)
    {
        return vec2(0.0);
    }

    float rSafe = clamp(r, planetRadius, atmRadius);
    float muSafe = clamp(mu, -1.0, 1.0);
    if (transmittance_ray_hits_planet(rSafe, muSafe, planetRadius))
    {
        return vec2(TRANSMITTANCE_MAX_AIRMASS);
    }

    ivec2 size = textureSize(lut, 0);
    float muTexel = 2.0 / max(float(size.x), 1.0);
    float horizonMu = transmittance_horizon_mu(rSafe, planetRadius);
    muSafe = clamp(max(muSafe, horizonMu + muTexel), -1.0, 1.0);

    float u = muSafe * 0.5 + 0.5;
    float v = clamp((rSafe - planetRadius) / max(atmRadius - planetRadius, 1.0e-4), 0.0, 1.0);
    return textureLod(lut, vec2(u, v), 0.0).rg;
}
