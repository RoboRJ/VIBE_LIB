#pragma once

namespace studentlib {

// velocityTolerance < 0 means "do not use velocity tolerance".
struct SettleConfig {
    double positionTolerance{0.5};
    double velocityTolerance{-1.0};
    double settleTimeSeconds{0.2};
    double timeoutSeconds{2.0};

    bool isValid() const;
};

}  // namespace studentlib