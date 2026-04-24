#pragma once

#include <optional>
#include <vector>

#include "studentlib/hardware/tracker_wheel.hpp"

namespace studentlib {

// Snapshot structs store already-read measurement values.
// They do not store devices or raw PROS objects.

struct IMUSnapshot {
    double headingRadians{0.0};
};

struct TrackerWheelSnapshot {
    TrackerAxis axis{TrackerAxis::LONGITUDINAL};
    double distanceInches{0.0};
};

struct DistanceSensorSnapshot {
    double distanceInches{0.0};
};

struct SensorSnapshot {
    std::optional<IMUSnapshot> imu{};
    std::vector<TrackerWheelSnapshot> trackerWheels{};
    std::vector<DistanceSensorSnapshot> distanceSensors{};
};

}  // namespace studentlib