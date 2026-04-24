#include "studentlib/estimation/mcl/mcl_config.hpp"

namespace studentlib {

bool MCLConfig::isValid() const {
    return particleCount > 0 &&
           translationNoiseStdDevInches >= 0.0 &&
           rotationNoiseStdDevRadians >= 0.0 &&
           distanceSensorNoiseStdDevInches >= 0.0 &&
           headingSensorNoiseStdDevRadians >= 0.0 &&
           resampleEffectiveCountThresholdRatio >= 0.0 &&
           resampleEffectiveCountThresholdRatio <= 1.0 &&
           randomParticleInjectionRatio >= 0.0 &&
           randomParticleInjectionRatio <= 1.0 &&
           extractedPoseSmoothingFactor >= 0.0 &&
           extractedPoseSmoothingFactor <= 1.0;
}

}  // namespace studentlib