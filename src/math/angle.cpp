#include "studentlib/math/angle.hpp"

#include <cmath>

namespace studentlib {

double degreesToRadians(double degrees) {
    return degrees * (kPi / 180.0);
}

double radiansToDegrees(double radians) {
    return radians * (180.0 / kPi);
}

double normalizeAngle(double angle_radians) {
    double wrapped = std::fmod(angle_radians + kPi, kTwoPi);

    if (wrapped < 0.0) {
        wrapped += kTwoPi;
    }

    return wrapped - kPi;
}

double normalizeAnglePositive(double angle_radians) {
    double wrapped = std::fmod(angle_radians, kTwoPi);

    if (wrapped < 0.0) {
        wrapped += kTwoPi;
    }

    return wrapped;
}

double smallestAngleDifference(double target_angle, double current_angle) {
    return normalizeAngle(target_angle - current_angle);
}

bool anglesNear(double a, double b, double tolerance) {
    return std::abs(smallestAngleDifference(a, b)) <= tolerance;
}

}  // namespace studentlib