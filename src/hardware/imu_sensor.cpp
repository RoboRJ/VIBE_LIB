#include "studentlib/hardware/imu_sensor.hpp"

#include "studentlib/math/angle.hpp"

namespace studentlib {

IMUSensor::IMUSensor(std::int8_t port, double yawOffsetRadians)
    : imu_(port),
      yaw_offset_radians_(yawOffsetRadians) {}

void IMUSensor::reset() {
    imu_.reset();
}

bool IMUSensor::isCalibrating() const {
    return imu_.is_calibrating();
}

double IMUSensor::getHeadingRadians() const {
    return normalizeAngle(getRawHeadingRadians() + yaw_offset_radians_);
}

double IMUSensor::getRawHeadingRadians() const {
    const double raw_degrees = imu_.get_rotation();
    return degreesToRadians(raw_degrees);
}

void IMUSensor::setYawOffset(double yawOffsetRadians) {
    yaw_offset_radians_ = yawOffsetRadians;
}

double IMUSensor::getYawOffset() const {
    return yaw_offset_radians_;
}

}  // namespace studentlib