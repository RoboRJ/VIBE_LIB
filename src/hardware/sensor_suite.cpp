#include "studentlib/hardware/sensor_suite.hpp"

namespace studentlib {

void SensorSuite::setIMU(IMUSensor imu) {
    imu_.emplace(std::move(imu));
}

bool SensorSuite::hasIMU() const {
    return imu_.has_value();
}

IMUSensor* SensorSuite::getIMU() {
    if (!imu_.has_value()) {
        return nullptr;
    }

    return &(*imu_);
}

const IMUSensor* SensorSuite::getIMU() const {
    if (!imu_.has_value()) {
        return nullptr;
    }

    return &(*imu_);
}

void SensorSuite::addTrackerWheel(TrackerWheel tracker) {
    tracker_wheels_.push_back(std::move(tracker));
}

std::vector<TrackerWheel>& SensorSuite::getTrackerWheels() {
    return tracker_wheels_;
}

const std::vector<TrackerWheel>& SensorSuite::getTrackerWheels() const {
    return tracker_wheels_;
}

void SensorSuite::addDistanceSensor(DistanceSensor sensor) {
    distance_sensors_.push_back(std::move(sensor));
}

std::vector<DistanceSensor>& SensorSuite::getDistanceSensors() {
    return distance_sensors_;
}

const std::vector<DistanceSensor>& SensorSuite::getDistanceSensors() const {
    return distance_sensors_;
}

void SensorSuite::reset() {
    if (imu_.has_value()) {
        imu_->reset();
    }

    for (auto& tracker : tracker_wheels_) {
        tracker.reset();
    }
}

}  // namespace studentlib