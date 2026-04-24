#include "studentlib/hardware/rotation_sensor.hpp"

#include "studentlib/math/angle.hpp"

namespace studentlib {

namespace {
constexpr double kCentidegreesToDegrees = 0.01;
}  // namespace

RotationSensor::RotationSensor(std::int8_t port, bool reversed)
    : sensor_(port),
      reversed_(reversed),
      last_angle_radians_(0.0) {}

void RotationSensor::reset() {
    sensor_.reset_position();
    last_angle_radians_ = 0.0;
}

double RotationSensor::getAngleRadians() const {
    const double position_centidegrees = sensor_.get_position();
    const double position_degrees = position_centidegrees * kCentidegreesToDegrees;
    return reversed_? -degreesToRadians(position_degrees) : degreesToRadians(position_degrees);
}

double RotationSensor::getDeltaRadians() {
    const double current_angle_radians = getAngleRadians();
    const double delta_radians = current_angle_radians - last_angle_radians_;
    last_angle_radians_ = current_angle_radians;
    return delta_radians;
}

bool RotationSensor::isReversed() const {
    return reversed_;
}

}  // namespace studentlib