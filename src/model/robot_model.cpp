#include "studentlib/model/robot_model.hpp"

namespace studentlib {

RobotModel::RobotModel(const DrivetrainConfig& drivetrainConfig)
    : drivetrain_config_(drivetrainConfig) {}

void RobotModel::setDrivetrainConfig(const DrivetrainConfig& drivetrainConfig) {
    drivetrain_config_ = drivetrainConfig;
}

const DrivetrainConfig& RobotModel::getDrivetrainConfig() const {
    return drivetrain_config_;
}

void RobotModel::setMountedIMU(const MountedIMU& mountedImu) {
    mounted_imu_ = mountedImu;
}

bool RobotModel::hasMountedIMU() const {
    return mounted_imu_.has_value();
}

const std::optional<MountedIMU>& RobotModel::getMountedIMU() const {
    return mounted_imu_;
}

void RobotModel::addMountedTrackerWheel(const MountedTrackerWheel& mountedTrackerWheel) {
    mounted_tracker_wheels_.push_back(mountedTrackerWheel);
}

const std::vector<MountedTrackerWheel>& RobotModel::getMountedTrackerWheels() const {
    return mounted_tracker_wheels_;
}

void RobotModel::addMountedDistanceSensor(const MountedDistanceSensor& mountedDistanceSensor) {
    mounted_distance_sensors_.push_back(mountedDistanceSensor);
}

const std::vector<MountedDistanceSensor>& RobotModel::getMountedDistanceSensors() const {
    return mounted_distance_sensors_;
}

}  // namespace studentlib