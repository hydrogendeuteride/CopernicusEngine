#pragma once

#include "game/orbit/kepler/kepler_types.h"

#include "orbitsim/math.hpp"

namespace Game
{
    struct KeplerOrbitMetrics
    {
        bool valid{false};
        orbitsim::BodyId primary_body_id{orbitsim::kInvalidBodyId};
        orbitsim::KeplerOrbitRegime regime{orbitsim::KeplerOrbitRegime::Unknown};
        double mu_m3_s2{0.0};
        double semi_major_axis_m{0.0};
        double eccentricity{0.0};
        double inclination_rad{0.0};
        double period_s{0.0};
        double mean_motion_radps{0.0};
        double periapsis_radius_m{0.0};
        double apoapsis_radius_m{0.0};
        bool has_apoapsis{false};
        orbitsim::Vec3 periapsis_rel_m{0.0, 0.0, 0.0};
        orbitsim::Vec3 apoapsis_rel_m{0.0, 0.0, 0.0};
    };

    [[nodiscard]] KeplerOrbitMetrics compute_kepler_orbit_metrics(const KeplerOrbitArc &arc);
} // namespace Game
