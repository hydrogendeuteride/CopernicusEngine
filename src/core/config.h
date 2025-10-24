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
inline constexpr float kShadowCSMFar = 800.0f;
// Shadow map resolution used for stabilization (texel snapping). Must match actual image size.
inline constexpr float kShadowMapResolution = 2048.0f;
// Clipmap shadow configuration (used when cascades operate in clipmap mode)
// Base coverage radius of level 0 around the camera (world units). Each level doubles the radius.
inline constexpr float kShadowClipBaseRadius = 20.0f;
// Pullback distance of the light eye from the clipmap center along the light direction (world units)
inline constexpr float kShadowClipLightPullback = 160.0f;
// Additional Z padding for the orthographic frustum along light direction
inline constexpr float kShadowClipZPadding = 80.0f;
