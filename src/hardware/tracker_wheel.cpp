#include "studentlib/hardware/tracker_wheel.hpp"

namespace studentlib {

TrackerWheel::TrackerWheel(RotationSensor rotationSensor,
                           double wheelRadiusInches,
                           double gearRatio,
                           TrackerAxis axis)
    : rotation_sensor_(std::move(rotationSensor)),
      wheel_radius_inches_(wheelRadiusInches),
      gear_ratio_(gearRatio),
      axis_(axis),
      last_distance_inches_(0.0) {}

void TrackerWheel::reset() {
    rotation_sensor_.reset();
    last_distance_inches_ = 0.0;
}

double TrackerWheel::getDistanceTraveled() const {
    const double sensor_angle_radians = rotation_sensor_.getAngleRadians();
    const double wheel_angle_radians =
        wheelAngleRadiansFromSensorAngle(sensor_angle_radians);

    return wheel_radius_inches_ * wheel_angle_radians;
}

double TrackerWheel::getDeltaDistance() {
    const double current_distance_inches = getDistanceTraveled();
    const double delta_distance_inches = current_distance_inches - last_distance_inches_;
    last_distance_inches_ = current_distance_inches;
    return delta_distance_inches;
}

double TrackerWheel::getWheelRadius() const {
    return wheel_radius_inches_;
}

double TrackerWheel::getGearRatio() const {
    return gear_ratio_;
}

TrackerAxis TrackerWheel::getAxis() const {
    return axis_;
}

const RotationSensor& TrackerWheel::getRotationSensor() const {
    return rotation_sensor_;
}

RotationSensor& TrackerWheel::getRotationSensor() {
    return rotation_sensor_;
}

double TrackerWheel::wheelAngleRadiansFromSensorAngle(double sensorAngleRadians) const {
    return sensorAngleRadians * gear_ratio_;
}

}  // namespace studentlib