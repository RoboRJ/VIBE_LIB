#pragma once

#include <optional>
#include <vector>

#include "studentlib/model/drivetrain_config.hpp"
#include "studentlib/model/sensor_mounts.hpp"

namespace studentlib {

class RobotModel {
public:
    RobotModel() = default;
    explicit RobotModel(const DrivetrainConfig& drivetrainConfig);

    void setDrivetrainConfig(const DrivetrainConfig& drivetrainConfig);
    const DrivetrainConfig& getDrivetrainConfig() const;

    void setMountedIMU(const MountedIMU& mountedImu);
    bool hasMountedIMU() const;
    const std::optional<MountedIMU>& getMountedIMU() const;

    void addMountedTrackerWheel(const MountedTrackerWheel& mountedTrackerWheel);
    const std::vector<MountedTrackerWheel>& getMountedTrackerWheels() const;

    void addMountedDistanceSensor(const MountedDistanceSensor& mountedDistanceSensor);
    const std::vector<MountedDistanceSensor>& getMountedDistanceSensors() const;

private:
    DrivetrainConfig drivetrain_config_{};
    std::optional<MountedIMU> mounted_imu_{};
    std::vector<MountedTrackerWheel> mounted_tracker_wheels_{};
    std::vector<MountedDistanceSensor> mounted_distance_sensors_{};
};

}  // namespace studentlib