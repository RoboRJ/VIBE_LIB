#pragma once

namespace studentlib {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double degreesToRadians(double degrees);
double radiansToDegrees(double radians);

double normalizeAngle(double angle_radians);
double normalizeAnglePositive(double angle_radians);

double smallestAngleDifference(double target_angle, double current_angle);
bool anglesNear(double a, double b, double tolerance);

}  // namespace studentlib