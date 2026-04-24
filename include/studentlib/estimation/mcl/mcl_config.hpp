#pragma once

#include <cstddef>

namespace studentlib {

struct MCLConfig {
    std::size_t particleCount{200};
    double translationNoiseStdDevInches{0.25};
    double rotationNoiseStdDevRadians{0.02};
    double distanceSensorNoiseStdDevInches{1.0};
    double headingSensorNoiseStdDevRadians{0.05};
    double resampleEffectiveCountThresholdRatio{0.5};
    double randomParticleInjectionRatio{0.0};
    double extractedPoseSmoothingFactor{0.0};

    bool isValid() const;
};

}  // namespace studentlib