#include "studentlib/estimation/mcl/mcl_estimator.hpp"

#include <algorithm>
#include <cmath>

#include "studentlib/math/angle.hpp"
#include "studentlib/math/math_util.hpp"

namespace studentlib {

MCLEstimator::MCLEstimator(
    SensorSuite& sensors,
    const RobotModel& robotModel,
    const FieldMap& fieldMap,
    const MCLConfig& config)
    : sensors_(sensors),
      robot_model_(robotModel),
      field_map_(fieldMap),
      config_(config),
      motion_model_(config_),
      sensor_model_(config_),
      resampler_(config_) {
    initializeParticlesAtPose(Pose2D{});
}

Pose2D MCLEstimator::getPose() const {
    return extracted_pose_;
}

void MCLEstimator::setPose(const Pose2D& pose) {
    Pose2D normalized_pose = pose;
    normalized_pose.theta = normalizeAngle(normalized_pose.theta);

    extracted_pose_ = normalized_pose;
    velocity_ = Twist2D{};
    previous_snapshot_.reset();

    initializeParticlesAtPose(normalized_pose);
}

Twist2D MCLEstimator::getVelocity() const {
    return velocity_;
}

void MCLEstimator::reset() {
    extracted_pose_ = Pose2D{};
    velocity_ = Twist2D{};
    previous_snapshot_.reset();

    initializeParticlesAtPose(Pose2D{});
}

void MCLEstimator::step(double dt) {
    const SensorSnapshot current_snapshot = captureSnapshot();

    // First step after construction, reset, or setPose establishes the
    // baseline snapshot and produces no motion jump.
    if (!previous_snapshot_.has_value()) {
        previous_snapshot_ = current_snapshot;
        velocity_ = Twist2D{};
        return;
    }

    const MotionDelta delta = computeMotionDelta(*previous_snapshot_, current_snapshot);

    for (auto& particle : particles_) {
        motion_model_.apply(particle, delta);
    }

    for (auto& particle : particles_) {
        particle.weight = sensor_model_.likelihood(
            particle.pose,
            current_snapshot,
            robot_model_,
            field_map_);
    }

    resampler_.normalizeWeights(particles_);

    const double effective_particle_count =
        resampler_.computeEffectiveParticleCount(particles_);

    const double resample_threshold =
        config_.resampleEffectiveCountThresholdRatio *
        static_cast<double>(particles_.size());

    if (effective_particle_count < resample_threshold) {
        resampler_.resample(particles_, field_map_);
    }

    const Pose2D previous_pose = extracted_pose_;
    Pose2D new_pose = extractPose();

    if (config_.extractedPoseSmoothingFactor > 0.0) {
        const double alpha = clamp(config_.extractedPoseSmoothingFactor, 0.0, 1.0);

        const double theta_error =
            normalizeAngle(new_pose.theta - previous_pose.theta);

        new_pose.x = previous_pose.x + alpha * (new_pose.x - previous_pose.x);
        new_pose.y = previous_pose.y + alpha * (new_pose.y - previous_pose.y);
        new_pose.theta = normalizeAngle(previous_pose.theta + alpha * theta_error);
    }

    extracted_pose_ = new_pose;

    if (dt > 0.0) {
        velocity_.vx = (extracted_pose_.x - previous_pose.x) / dt;
        velocity_.vy = (extracted_pose_.y - previous_pose.y) / dt;
        velocity_.omega =
            normalizeAngle(extracted_pose_.theta - previous_pose.theta) / dt;
    } else {
        velocity_ = Twist2D{};
    }

    previous_snapshot_ = current_snapshot;
}

void MCLEstimator::initializeParticlesAtPose(const Pose2D& pose) {
    const std::size_t particle_count =
        (config_.particleCount > 0) ? config_.particleCount : 1;

    particles_.clear();
    particles_.reserve(particle_count);

    const double uniform_weight = 1.0 / static_cast<double>(particle_count);

    for (std::size_t i = 0; i < particle_count; ++i) {
        Particle particle;
        particle.pose = pose;
        particle.weight = uniform_weight;
        particles_.push_back(particle);
    }
}

SensorSnapshot MCLEstimator::captureSnapshot() const {
    SensorSnapshot snapshot;

    const auto& mounted_imu = robot_model_.getMountedIMU();

    if (mounted_imu.has_value() && mounted_imu->imu != nullptr) {
        const double heading = normalizeAngle(
            mounted_imu->imu->getRawHeadingRadians() +
            mounted_imu->yawOffsetRadians);

        snapshot.imu = IMUSnapshot{heading};
    } else {
        const IMUSensor* imu =
            static_cast<const SensorSuite&>(sensors_).getIMU();

        if (imu != nullptr) {
            snapshot.imu = IMUSnapshot{imu->getHeadingRadians()};
        }
    }

    const auto& tracker_wheels =
        static_cast<const SensorSuite&>(sensors_).getTrackerWheels();

    for (const auto& tracker : tracker_wheels) {
        snapshot.trackerWheels.push_back(
            TrackerWheelSnapshot{tracker.getAxis(), tracker.getDistanceTraveled()});
    }

    const auto& distance_sensors =
        static_cast<const SensorSuite&>(sensors_).getDistanceSensors();

    for (const auto& distance_sensor : distance_sensors) {
        const std::optional<double> reading = distance_sensor.getDistanceInches();

        if (reading.has_value()) {
            snapshot.distanceSensors.push_back(
                DistanceSensorSnapshot{*reading});
        }
    }

    return snapshot;
}

MotionDelta MCLEstimator::computeMotionDelta(
    const SensorSnapshot& previous,
    const SensorSnapshot& current) const {

    MotionDelta delta;

    double longitudinal_sum = 0.0;
    double lateral_sum = 0.0;
    int longitudinal_count = 0;
    int lateral_count = 0;

    const std::size_t tracker_count = std::min(
        previous.trackerWheels.size(),
        current.trackerWheels.size());

    for (std::size_t i = 0; i < tracker_count; ++i) {
        const TrackerWheelSnapshot& previous_tracker = previous.trackerWheels[i];
        const TrackerWheelSnapshot& current_tracker = current.trackerWheels[i];

        if (previous_tracker.axis != current_tracker.axis) {
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

    if (previous.imu.has_value() && current.imu.has_value()) {
        delta.dthetaRadians = normalizeAngle(
            current.imu->headingRadians - previous.imu->headingRadians);
    }

    return delta;
}

Pose2D MCLEstimator::extractPose() const {
    if (particles_.empty()) {
        return Pose2D{};
    }

    double weight_sum = 0.0;
    double x_sum = 0.0;
    double y_sum = 0.0;
    double sin_sum = 0.0;
    double cos_sum = 0.0;

    for (const auto& particle : particles_) {
        weight_sum += particle.weight;
        x_sum += particle.weight * particle.pose.x;
        y_sum += particle.weight * particle.pose.y;
        sin_sum += particle.weight * std::sin(particle.pose.theta);
        cos_sum += particle.weight * std::cos(particle.pose.theta);
    }

    if (weight_sum <= 0.0) {
        return Pose2D{};
    }

    Pose2D pose;
    pose.x = x_sum / weight_sum;
    pose.y = y_sum / weight_sum;
    pose.theta = normalizeAngle(std::atan2(sin_sum, cos_sum));

    return pose;
}

}  // namespace studentlib