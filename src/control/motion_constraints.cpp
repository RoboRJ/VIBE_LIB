#include "studentlib/control/motion_constraints.hpp"

namespace studentlib {

bool MotionConstraints::isValid() const {
    return maxSpeed >= 0.0 &&
           minSpeed >= 0.0 &&
           minSpeed <= maxSpeed &&
           maxTurnSpeed >= 0.0 &&
           maxAcceleration >= 0.0;
}

}  // namespace studentlib