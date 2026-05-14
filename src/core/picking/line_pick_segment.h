#pragma once

#include "core/world.h"

#include <cstdint>
#include <limits>

namespace Picking
{
    struct LinePickPayload
    {
        uint32_t type{0};
        uint32_t role{0};
        uint64_t primary_id{0};
        uint64_t secondary_id{0};
    };

    struct LinePickSegmentData
    {
        WorldVec3 a_world{0.0, 0.0, 0.0};
        WorldVec3 b_world{0.0, 0.0, 0.0};
        double a_time_s = std::numeric_limits<double>::quiet_NaN();
        double b_time_s = std::numeric_limits<double>::quiet_NaN();
    };
} // namespace Picking
