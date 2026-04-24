#pragma once

#include "studentlib/hardware/distance_sensor.hpp"
#include "studentlib/hardware/imu_sensor.hpp"
#include "studentlib/hardware/tracker_wheel.hpp"
#include "studentlib/math/point.hpp"
#include "studentlib/math/pose.hpp"

namespace studentlib {

// These structs do not own hardware.
// The actual sensor objects live elsewhere, such as SensorSuite.

struct MountedIMU {
    IMUSensor* imu{nullptr};
    double yawOffsetRadians{0.0};
};

struct MountedTrackerWheel {
    TrackerWheel* tracker{nullptr};
    Point2D positionInRobotFrame{};
};

struct MountedDistanceSensor {
    DistanceSensor* sensor{nullptr};
    Pose2D poseInRobotFrame{};
    double maxRangeInches{0.0};
    double beamAngleOffsetRadians{0.0};
};

}  // namespace studentlib