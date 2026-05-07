#include "game/orbit/kepler/kepler_debug.h"

namespace Game
{
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
