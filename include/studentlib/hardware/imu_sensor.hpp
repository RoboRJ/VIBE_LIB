#pragma once

#include <cstdint>

#include "pros/imu.hpp"

namespace studentlib {

class IMUSensor {
public:
    explicit IMUSensor(std::int8_t port, double yawOffsetRadians = 0.0);

    void reset();
    bool isCalibrating() const;

    double getHeadingRadians() const;
    double getRawHeadingRadians() const;

    void setYawOffset(double yawOffsetRadians);
    double getYawOffset() const;

private:
    pros::Imu imu_;
    double yaw_offset_radians_{0.0};
};

}  // namespace studentlib