#pragma once

#include "orbitsim/types.hpp"

#include <glm/geometric.hpp>

#include <cmath>

namespace Game::KeplerManeuverMath
{
    inline double safe_length(const glm::dvec3 &v)
    {
        const double len2 = glm::dot(v, v);
        if (!(len2 > 0.0) || !std::isfinite(len2))
        {
            return 0.0;
        }
        return std::sqrt(len2);
    }

    inline orbitsim::Vec3 normalized_or(const orbitsim::Vec3 &v,
                                        const orbitsim::Vec3 &fallback,
                                        const double min_length = 0.0)
    {
        const double len = safe_length(v);
        if (!(len > min_length) || !std::isfinite(len))
        {
            return fallback;
        }
        return v / len;
    }
} // namespace Game::KeplerManeuverMath
