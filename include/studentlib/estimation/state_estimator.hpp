#pragma once

#include "studentlib/math/pose.hpp"
#include "studentlib/math/twist.hpp"

namespace studentlib {

class StateEstimator {
public:
    virtual Pose2D getPose() const = 0;
    virtual void setPose(const Pose2D& pose) = 0;
    virtual Twist2D getVelocity() const = 0;
    virtual void reset() = 0;
    virtual void step(double dt) = 0;
    virtual ~StateEstimator() = default;
};

}  // namespace studentlib