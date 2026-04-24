#pragma once

namespace studentlib {

struct MotionConstraints {
    double maxSpeed{1.0};
    double minSpeed{0.0};
    double maxTurnSpeed{1.0};
    double maxAcceleration{0.0};

    bool isValid() const;
};

}  // namespace studentlib