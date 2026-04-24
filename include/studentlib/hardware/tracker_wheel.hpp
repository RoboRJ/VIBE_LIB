#pragma once

#include <cstdint>

#include "studentlib/hardware/rotation_sensor.hpp"

namespace studentlib {

enum class TrackerAxis {
    LONGITUDINAL,
    LATERAL
};

class TrackerWheel {
public:
    TrackerWheel(RotationSensor rotationSensor,
                 double wheelRadiusInches,
                 double gearRatio,
                 TrackerAxis axis);

    void reset();

    double getDistanceTraveled() const;
    double getDeltaDistance();

    double getWheelRadius() const;
    double getGearRatio() const;
    TrackerAxis getAxis() const;

    const RotationSensor& getRotationSensor() const;
    RotationSensor& getRotationSensor();

private:
    RotationSensor rotation_sensor_;
    double wheel_radius_inches_{0.0};
    double gear_ratio_{1.0};
    TrackerAxis axis_{TrackerAxis::LONGITUDINAL};
    double last_distance_inches_{0.0};

    double wheelAngleRadiansFromSensorAngle(double sensorAngleRadians) const;
};

}  // namespace studentlib