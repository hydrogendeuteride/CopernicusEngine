#include "game/orbit/kepler/kepler_arc_info.h"

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

    KeplerArcMetrics compute_kepler_arc_metrics(const KeplerOrbitArc &arc)
    {
        KeplerArcMetrics out{};
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

    const char *kepler_orbit_status_name(const KeplerOrbitStatus status)
    {
        switch (status)
        {
            case KeplerOrbitStatus::Ok:
                return "Ok";
            case KeplerOrbitStatus::InvalidInput:
                return "InvalidInput";
            case KeplerOrbitStatus::InvalidSimulation:
                return "InvalidSimulation";
            case KeplerOrbitStatus::InvalidSubjectState:
                return "InvalidSubjectState";
            case KeplerOrbitStatus::PrimaryUnavailable:
                return "PrimaryUnavailable";
            case KeplerOrbitStatus::InvalidPrimary:
                return "InvalidPrimary";
            case KeplerOrbitStatus::InvalidArc:
                return "InvalidArc";
            case KeplerOrbitStatus::PrimaryMismatch:
                return "PrimaryMismatch";
            case KeplerOrbitStatus::EphemerisUnavailable:
                return "EphemerisUnavailable";
            case KeplerOrbitStatus::ContinuityFailed:
                return "ContinuityFailed";
            case KeplerOrbitStatus::PropagationFailed:
                return "PropagationFailed";
            case KeplerOrbitStatus::SampleBudgetExceeded:
                return "SampleBudgetExceeded";
            case KeplerOrbitStatus::NoSamples:
                return "NoSamples";
        }
        return "Unknown";
    }

    const char *kepler_orbit_regime_name(const orbitsim::KeplerOrbitRegime regime)
    {
        switch (regime)
        {
            case orbitsim::KeplerOrbitRegime::Unknown:
                return "Unknown";
            case orbitsim::KeplerOrbitRegime::Elliptic:
                return "Elliptic";
            case orbitsim::KeplerOrbitRegime::NearParabolic:
                return "NearParabolic";
            case orbitsim::KeplerOrbitRegime::Hyperbolic:
                return "Hyperbolic";
        }
        return "Unknown";
    }
} // namespace Game
