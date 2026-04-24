#include "studentlib/estimation/odometry_estimator.hpp"

#include <algorithm>
#include <cmath>

#include "studentlib/math/angle.hpp"

namespace studentlib {

OdometryEstimator::OdometryEstimator(SensorSuite& sensors, const RobotModel& robotModel)
    : sensors_(sensors),
      robot_model_(robotModel) {}

Pose2D OdometryEstimator::getPose() const {
    return pose_;
}

void OdometryEstimator::setPose(const Pose2D& pose) {
    pose_ = pose;
    pose_.theta = normalizeAngle(pose_.theta);
}

Twist2D OdometryEstimator::getVelocity() const {
    return velocity_;
}

void OdometryEstimator::reset() {
    // Reset only estimator state.
    // This avoids silently recalibrating hardware from a localization helper.
    pose_ = Pose2D{};
    velocity_ = Twist2D{};
    previous_snapshot_.reset();
}

void OdometryEstimator::step(double dt) {
    const SensorSnapshot current = captureSnapshot();

    // First step after construction or reset establishes the baseline
    // without producing a jump in pose.
    if (!previous_snapshot_.has_value()) {
        previous_snapshot_ = current;
        velocity_ = Twist2D{};
        return;
    }

    const MotionDelta delta = computeMotionDelta(current);

    const double theta_mid = pose_.theta + (delta.dthetaRadians * 0.5);

    // Rotate the body-frame increment into the world frame
    // using midpoint heading.
    const double dx_world =
        delta.dxLocalInches * std::cos(theta_mid) -
        delta.dyLocalInches * std::sin(theta_mid);

    const double dy_world =
        delta.dxLocalInches * std::sin(theta_mid) +
        delta.dyLocalInches * std::cos(theta_mid);

    pose_.x += dx_world;
    pose_.y += dy_world;
    pose_.theta = normalizeAngle(pose_.theta + delta.dthetaRadians);

    if (dt > 0.0) {
        velocity_.vx = dx_world / dt;
        velocity_.vy = dy_world / dt;
        velocity_.omega = delta.dthetaRadians / dt;
    } else {
        velocity_ = Twist2D{};
    }

    previous_snapshot_ = current;
}

SensorSnapshot OdometryEstimator::captureSnapshot() const {
    SensorSnapshot snapshot;

    const auto& mounted_imu = robot_model_.getMountedIMU();

    // Prefer model metadata for IMU heading if it is available.
    // In v1 this lets the model layer supply a robot-frame yaw offset.
    if (mounted_imu.has_value() && mounted_imu->imu != nullptr) {
        const double heading =
            normalizeAngle(mounted_imu->imu->getRawHeadingRadians() +
                           mounted_imu->yawOffsetRadians);
        snapshot.imu = IMUSnapshot{heading};
    } else {
        const IMUSensor* imu = sensors_.getIMU();
        if (imu != nullptr) {
            snapshot.imu = IMUSnapshot{imu->getHeadingRadians()};
        }
    }

    for (const auto& tracker : sensors_.getTrackerWheels()) {
        snapshot.trackerWheels.push_back(
            TrackerWheelSnapshot{tracker.getAxis(), tracker.getDistanceTraveled()});
    }

    for (const auto& distance_sensor : sensors_.getDistanceSensors()) {
        const std::optional<double> reading = distance_sensor.getDistanceInches();
        if (reading.has_value()) {
            snapshot.distanceSensors.push_back(
                DistanceSensorSnapshot{*reading});
        }
    }

    return snapshot;
}

MotionDelta OdometryEstimator::computeMotionDelta(const SensorSnapshot& current) const {
    MotionDelta delta;

    if (!previous_snapshot_.has_value()) {
        return delta;
    }

    const SensorSnapshot& previous = *previous_snapshot_;

    double longitudinal_sum = 0.0;
    double lateral_sum = 0.0;
    int longitudinal_count = 0;
    int lateral_count = 0;

    const std::size_t tracker_count =
        std::min(current.trackerWheels.size(), previous.trackerWheels.size());

    // In v1, tracker wheels are treated as direct body-frame displacement sensors.
    // This intentionally skips rotational compensation from wheel mount offsets
    // to keep the first odometry pass readable.
    for (std::size_t i = 0; i < tracker_count; ++i) {
        const TrackerWheelSnapshot& current_tracker = current.trackerWheels[i];
        const TrackerWheelSnapshot& previous_tracker = previous.trackerWheels[i];

        if (current_tracker.axis != previous_tracker.axis) {
            continue;
        }

        const double tracker_delta =
            current_tracker.distanceInches - previous_tracker.distanceInches;

        if (current_tracker.axis == TrackerAxis::LONGITUDINAL) {
            longitudinal_sum += tracker_delta;
            ++longitudinal_count;
        } else if (current_tracker.axis == TrackerAxis::LATERAL) {
            lateral_sum += tracker_delta;
            ++lateral_count;
        }
    }

    if (longitudinal_count > 0) {
        delta.dxLocalInches =
            longitudinal_sum / static_cast<double>(longitudinal_count);
    }

    if (lateral_count > 0) {
        delta.dyLocalInches =
            lateral_sum / static_cast<double>(lateral_count);
    }

    if (current.imu.has_value() && previous.imu.has_value()) {
        delta.dthetaRadians = normalizeAngle(
            current.imu->headingRadians - previous.imu->headingRadians);
    } else {
        delta.dthetaRadians = 0.0;
    }

    return delta;
}

}  // namespace studentlib