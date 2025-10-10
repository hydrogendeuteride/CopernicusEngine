#pragma once

// Centralized engine configuration flags
#ifdef NDEBUG
inline constexpr bool kUseValidationLayers = false;
#else
inline constexpr bool kUseValidationLayers = true;
#endif
