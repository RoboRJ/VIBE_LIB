#include "studentlib/estimation/mcl/sensor_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "studentlib/math/angle.hpp"
#include "studentlib/math/math_util.hpp"

namespace studentlib {

namespace {
constexpr double kMinimumSafeStdDev = 1e-9;
constexpr double kMinimumSafeLikelihood = 1e-12;
constexpr double kMinimumSafeLogLikelihood = -700.0;
constexpr double kSqrtTwoPi = 2.5066282746310002;
}

SensorModel::SensorModel(const MCLConfig& config)
    : config_(config) {}

double SensorModel::likelihood(
    const Pose2D& particlePose,
    const SensorSnapshot& snapshot,
    const RobotModel& robotModel,
    const FieldMap& fieldMap) const {

    bool used_measurement = false;
    double log_likelihood = 0.0;

    if (snapshot.imu.has_value() && robotModel.hasMountedIMU()) {
        // In this project, the snapshot IMU heading is treated as a robot-frame
        // heading measurement. Any mount yaw offset is applied during snapshot capture,
        // so the expected heading here is just the particle's robot heading.
        const double heading_error =
            normalizeAngle(snapshot.imu->headingRadians - particlePose.theta);

        log_likelihood += gaussianLogLikelihood(
            heading_error,
            config_.headingSensorNoiseStdDevRadians);

        used_measurement = true;
    }

    const auto& mounted_distance_sensors = robotModel.getMountedDistanceSensors();
    const std::size_t distance_count = std::min(
        snapshot.distanceSensors.size(),
        mounted_distance_sensors.size());

    for (std::size_t i = 0; i < distance_count; ++i) {
        const MountedDistanceSensor& mounted = mounted_distance_sensors[i];

        if (mounted.sensor == nullptr || mounted.maxRangeInches <= 0.0) {
            continue;
        }

        // Current SensorSnapshot does not identify which mounted distance sensor
        // produced which reading. In this first pass, readings are assumed to
        // correspond to mounted distance sensors by order.
        const Point2D sensor_origin_local{
            mounted.poseInRobotFrame.x,
            mounted.poseInRobotFrame.y};

        const Point2D sensor_origin_world =
            transformPoint(particlePose, sensor_origin_local);

        const double beam_direction_world = normalizeAngle(
            particlePose.theta +
            mounted.poseInRobotFrame.theta +
            mounted.beamAngleOffsetRadians);

        const std::optional<double> predicted_distance = fieldMap.raycastDistance(
            sensor_origin_world,
            beam_direction_world,
            mounted.maxRangeInches);

        const double expected_distance =
            predicted_distance.has_value() ? *predicted_distance
                                           : mounted.maxRangeInches;

        const double measured_distance = snapshot.distanceSensors[i].distanceInches;
        const double distance_error = measured_distance - expected_distance;

        log_likelihood += gaussianLogLikelihood(
            distance_error,
            config_.distanceSensorNoiseStdDevInches);

        used_measurement = true;
    }

    if (!used_measurement) {
        return 1.0;
    }

    if (!std::isfinite(log_likelihood)) {
        return kMinimumSafeLikelihood;
    }

    if (log_likelihood < kMinimumSafeLogLikelihood) {
        log_likelihood = kMinimumSafeLogLikelihood;
    }

    const double value = std::exp(log_likelihood);

    if (!std::isfinite(value) || value <= 0.0) {
        return kMinimumSafeLikelihood;
    }

    return value;
}

double SensorModel::gaussianLogLikelihood(double error, double standardDeviation) const {
    const double sigma = std::max(standardDeviation, kMinimumSafeStdDev);
    const double z = error / sigma;

    return -0.5 * z * z - std::log(sigma * kSqrtTwoPi);
}

}  // namespace studentlib