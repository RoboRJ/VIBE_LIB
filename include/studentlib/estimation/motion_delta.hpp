#pragma once

namespace studentlib {

// MotionDelta is a robot-local (body-frame) increment over one estimator step.
// dxLocalInches is forward/backward motion in the robot frame.
// dyLocalInches is sideways motion in the robot frame.
// dthetaRadians is the heading change over the same step.
struct MotionDelta {
    double dxLocalInches{0.0};
    double dyLocalInches{0.0};
    double dthetaRadians{0.0};

    MotionDelta() = default;

    MotionDelta(double dxLocalInchesIn,
                double dyLocalInchesIn,
                double dthetaRadiansIn)
        : dxLocalInches(dxLocalInchesIn),
          dyLocalInches(dyLocalInchesIn),
          dthetaRadians(dthetaRadiansIn) {}
};

}  // namespace studentlib