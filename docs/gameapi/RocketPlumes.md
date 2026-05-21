# Rocket Plumes

The game API exposes a small fixed set of analytic rocket plume slots. The underlying renderer raymarches a procedural plume in plume-local space.

## Relevant Types

- `GameAPI::RocketPlumeSettings`
- `GameAPI::Engine::set_rocket_plumes_enabled(...)`
- `GameAPI::Engine::get_rocket_plume(...)`
- `GameAPI::Engine::set_rocket_plume(...)`
- `GameAPI::Engine::get_max_rocket_plumes()`
- `GameAPI::Engine::set_rocket_plume_noise_texture_path(...)`

See `src/core/game_api.h`.

## Coordinate Model

`worldToPlume` defines the plume's local frame:

- `+Z` is the exhaust direction
- `z = 0` is the nozzle exit plane
- the matrix is expressed in world space
- floating-origin compensation is applied later by the renderer

## Main Controls

- shape: `length`, `nozzleRadius`, `expansionAngleRad`, `radiusExp`
- emission: `intensity`, `coreColor`, `plumeColor`, `coreLength`, `coreStrength`
- falloff: `radialFalloff`, `axialFalloff`
- noise: `noiseStrength`, `noiseScale`, `noiseSpeed`
- shock diamonds: `shockStrength`, `shockFrequency`
- absorption: `softAbsorption`

## Typical Flow

1. enable the plume system
2. query the available slot count
3. fill one or more `RocketPlumeSettings`
4. update those settings every frame as the nozzle transform changes

```cpp
api.set_rocket_plumes_enabled(true);

GameAPI::RocketPlumeSettings plume{};
plume.enabled = true;
plume.length = 12.0f;
plume.nozzleRadius = 0.12f;
plume.intensity = 10.0f;

api.set_rocket_plume(0, plume);
```

## Noise Texture

The plume system also has a shared noise texture path:

```cpp
api.set_rocket_plume_noise_texture_path("vfx/noise/plume_noise.ktx2");
```

That path is asset-relative under the normal `assets/` rules.

## Related Docs

- [../GameAPI.md](../GameAPI.md)
- [Volumetrics.md](Volumetrics.md)
