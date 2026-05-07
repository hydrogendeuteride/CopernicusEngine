#include "game/orbit/kepler/kepler_orbit_metrics.h"

#include <cmath>

namespace Game
{
    namespace
    {
        orbitsim::KeplerOrbitRegime classify_regime(const double eccentricity)
        {
            if (!std::isfinite(eccentricity))
            {
                return orbitsim::KeplerOrbitRegime::Unknown;
            }
            constexpr double kNearParabolicEpsilon = 1.0e-8;
            if (eccentricity < 1.0 - kNearParabolicEpsilon)
            {
                return orbitsim::KeplerOrbitRegime::Elliptic;
            }
            if (eccentricity > 1.0 + kNearParabolicEpsilon)
            {
                return orbitsim::KeplerOrbitRegime::Hyperbolic;
            }
            return orbitsim::KeplerOrbitRegime::NearParabolic;
        }
    } // namespace

    KeplerOrbitMetrics compute_kepler_orbit_metrics(const KeplerOrbitArc &arc)
    {
        KeplerOrbitMetrics out{};
        if (!orbitsim::kepler_arc_valid(arc.arc))
        {
            return out;
        }

        const orbitsim::OrbitalElements elements =
                orbitsim::orbital_elements_from_relative_state(arc.arc.mu_m3_s2,
                                                               arc.arc.state0_relative.position_m,
                                                               arc.arc.state0_relative.velocity_mps);
        const orbitsim::OrbitScalars scalars =
                orbitsim::orbit_scalars_from_elements(arc.arc.mu_m3_s2, elements);
        const orbitsim::OrbitApsides apsides =
                orbitsim::apsides_from_relative_state(arc.arc.mu_m3_s2,
                                                      arc.arc.state0_relative.position_m,
                                                      arc.arc.state0_relative.velocity_mps);

        if (!std::isfinite(elements.eccentricity) || !apsides.valid)
        {
            return out;
        }

        out.valid = true;
        out.primary_body_id = arc.arc.primary_body_id;
        out.regime = classify_regime(elements.eccentricity);
        out.mu_m3_s2 = arc.arc.mu_m3_s2;
        out.semi_major_axis_m = elements.semi_major_axis_m;
        out.eccentricity = elements.eccentricity;
        out.inclination_rad = elements.inclination_rad;
        out.period_s = scalars.period_s;
        out.mean_motion_radps = scalars.mean_motion_radps;
        out.periapsis_radius_m = apsides.periapsis_radius_m;
        out.apoapsis_radius_m = apsides.apoapsis_radius_m;
        out.has_apoapsis = apsides.has_apoapsis;
        out.periapsis_rel_m = apsides.periapsis_rel_m;
        out.apoapsis_rel_m = apsides.apoapsis_rel_m;
        return out;
    }
} // namespace Game
