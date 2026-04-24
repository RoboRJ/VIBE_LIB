#pragma once

#include <optional>
#include <vector>

#include "studentlib/hardware/distance_sensor.hpp"
#include "studentlib/hardware/imu_sensor.hpp"
#include "studentlib/hardware/tracker_wheel.hpp"

namespace studentlib {

class SensorSuite {
public:
    SensorSuite() = default;

    void setIMU(IMUSensor imu);
    bool hasIMU() const;

    IMUSensor* getIMU();
    const IMUSensor* getIMU() const;

    void addTrackerWheel(TrackerWheel tracker);
    std::vector<TrackerWheel>& getTrackerWheels();
    const std::vector<TrackerWheel>& getTrackerWheels() const;

    void addDistanceSensor(DistanceSensor sensor);
    std::vector<DistanceSensor>& getDistanceSensors();
    const std::vector<DistanceSensor>& getDistanceSensors() const;

    void reset();

private:
    std::optional<IMUSensor> imu_;
    std::vector<TrackerWheel> tracker_wheels_;
    std::vector<DistanceSensor> distance_sensors_;
};

}  // namespace studentlib