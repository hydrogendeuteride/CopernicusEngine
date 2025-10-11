#pragma once

// Centralized engine configuration flags
#ifdef NDEBUG
inline constexpr bool kUseValidationLayers = false;
#else
inline constexpr bool kUseValidationLayers = true;
#endif

// Shadow mapping configuration
inline constexpr int kShadowCascadeCount = 4;
// Maximum shadow distance for CSM in view-space units
inline constexpr float kShadowCSMFar = 50.0f;
// Shadow map resolution used for stabilization (texel snapping). Must match actual image size.
inline constexpr float kShadowMapResolution = 2048.0f;
// Extra XY expansion for cascade footprint (safety against FOV/aspect changes)
inline constexpr float kShadowCascadeRadiusScale = 1.15f;
// Additive XY margin in world units (light-space) beyond scaled radius
inline constexpr float kShadowCascadeRadiusMargin = 10.0f;
