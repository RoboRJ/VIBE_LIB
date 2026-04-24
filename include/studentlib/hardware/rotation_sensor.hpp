#pragma once

#include <cstdint>

#include "pros/rotation.hpp"

namespace studentlib {

class RotationSensor {
public:
    explicit RotationSensor(std::int8_t port, bool reversed = false);

    void reset();

    double getAngleRadians() const;
    double getDeltaRadians();

    bool isReversed() const;

private:
    pros::Rotation sensor_; // might need to be mutable
    bool reversed_{false};
    double last_angle_radians_{0.0};
};

}  // namespace studentlib