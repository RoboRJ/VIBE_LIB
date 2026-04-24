#include "studentlib/robot/robot.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include "studentlib/core/time_util.hpp"
#include "studentlib/core/units.hpp"
#include "studentlib/drivetrain/tank_drive.hpp"
#include "studentlib/estimation/mcl/mcl_estimator.hpp"
#include "studentlib/estimation/odometry_estimator.hpp"
#include "studentlib/hardware/distance_sensor.hpp"
#include "studentlib/hardware/imu_sensor.hpp"
#include "studentlib/hardware/motor_group.hpp"
#include "studentlib/hardware/rotation_sensor.hpp"
#include "studentlib/hardware/tracker_wheel.hpp"

namespace studentlib {

namespace {

class MutexGuard {
public:
    explicit MutexGuard(pros::Mutex& mutex)
        : mutex_(mutex) {
        mutex_.take();
    }

    ~MutexGuard() {
        mutex_.give();
    }

private:
    pros::Mutex& mutex_;
};

std::uint32_t periodToMilliseconds(double seconds) {
    if (seconds <= 0.0) {
        return 1;
    }

    const double milliseconds = secondsToMilliseconds(seconds);

    if (milliseconds < 1.0) {
        return 1;
    }

    return static_cast<std::uint32_t>(milliseconds);
}

}  // namespace

Robot::Robot(
    const RobotConfig& robotConfig,
    const DrivetrainConfig& drivetrainConfig,
    const std::vector<int>& leftMotorPorts,
    const std::vector<int>& rightMotorPorts,
    const std::optional<RobotIMUConfig>& imuConfig,
    const std::vector<RobotTrackerConfig>& trackerConfigs,
    const std::vector<RobotDistanceSensorConfig>& distanceSensorConfigs,
    const std::optional<FieldMap>& fieldMap)
    : config_(robotConfig),
      robot_model_(drivetrainConfig),
      field_map_(fieldMap) {
    if (config_.localizationMode == LocalizationMode::MCL && !field_map_.has_value()) {
        // MCL needs a field map for measurement prediction.
        // In v1, the simplest safe behavior is to fall back to odometry.
        config_.localizationMode = LocalizationMode::ODOMETRY;
    }

    configureHardware(
        drivetrainConfig,
        leftMotorPorts,
        rightMotorPorts,
        imuConfig,
        trackerConfigs,
        distanceSensorConfigs);

    state_estimator_ = createStateEstimator();
    motion_manager_.emplace(*drivetrain_, *state_estimator_);
}

Robot::~Robot() {
    stop();
    shutdown_requested_.store(true);

    const double longest_period = std::max(
        config_.localizationPeriodSeconds,
        config_.motionPeriodSeconds);

    delayMilliseconds(periodToMilliseconds(longest_period) + 5U);

    localization_task_.reset();
    motion_task_.reset();
}

void Robot::initialize() {
    MutexGuard guard(runtime_mutex_);

    sensor_suite_.reset();

    if (state_estimator_) {
        state_estimator_->reset();
    }

    if (motion_manager_.has_value()) {
        motion_manager_->cancel();
    }

    if (drivetrain_) {
        drivetrain_->stop();
    }

    initialized_.store(true);
}

void Robot::start() {
    if (!initialized_.load()) {
        initialize();
    }

    if (!localization_task_) {
        localization_task_ =
            std::make_unique<pros::Task>(localizationTaskEntry, this);
    }

    if (!motion_task_) {
        motion_task_ =
            std::make_unique<pros::Task>(motionTaskEntry, this);
    }

    runtime_active_.store(true);
}

void Robot::stop() {
    runtime_active_.store(false);

    MutexGuard guard(runtime_mutex_);

    if (motion_manager_.has_value()) {
        motion_manager_->cancel();
    }

    if (drivetrain_) {
        drivetrain_->stop();
    }
}

Pose2D Robot::getPose() const {
    MutexGuard guard(runtime_mutex_);

    if (!state_estimator_) {
        return Pose2D{};
    }

    return state_estimator_->getPose();
}

void Robot::setPose(const Pose2D& pose) {
    MutexGuard guard(runtime_mutex_);

    if (state_estimator_) {
        state_estimator_->setPose(pose);
    }
}

void Robot::resetPose() {
    MutexGuard guard(runtime_mutex_);

    if (state_estimator_) {
        state_estimator_->reset();
    }
}

void Robot::moveToPose(const Pose2D& target, const MoveToPoseParams& params) {
    startMoveToPose(target, params);
    waitUntilSettled();
}

void Robot::turnToPoint(const Point2D& target, const TurnToPointParams& params) {
    startTurnToPoint(target, params);
    waitUntilSettled();
}

void Robot::turnToHeading(double headingRadians, const TurnToHeadingParams& params) {
    startTurnToHeading(headingRadians, params);
    waitUntilSettled();
}

void Robot::driveToPoint(const Point2D& target, const DriveToPointParams& params) {
    startDriveToPoint(target, params);
    waitUntilSettled();
}

void Robot::startMoveToPose(const Pose2D& target, const MoveToPoseParams& params) {
    start();

    MutexGuard guard(runtime_mutex_);

    if (motion_manager_.has_value()) {
        motion_manager_->startMoveToPose(target, params);
    }
}

void Robot::startTurnToPoint(const Point2D& target, const TurnToPointParams& params) {
    start();

    MutexGuard guard(runtime_mutex_);

    if (motion_manager_.has_value()) {
        motion_manager_->startTurnToPoint(target, params);
    }
}

void Robot::startTurnToHeading(double headingRadians, const TurnToHeadingParams& params) {
    start();

    MutexGuard guard(runtime_mutex_);

    if (motion_manager_.has_value()) {
        motion_manager_->startTurnToHeading(headingRadians, params);
    }
}

void Robot::startDriveToPoint(const Point2D& target, const DriveToPointParams& params) {
    start();

    MutexGuard guard(runtime_mutex_);

    if (motion_manager_.has_value()) {
        motion_manager_->startDriveToPoint(target, params);
    }
}

MotionStatus Robot::getMotionStatus() const {
    MutexGuard guard(runtime_mutex_);

    if (!motion_manager_.has_value()) {
        return MotionStatus::IDLE;
    }

    return motion_manager_->getStatus();
}

bool Robot::isMotionComplete() const {
    MutexGuard guard(runtime_mutex_);

    if (!motion_manager_.has_value()) {
        return true;
    }

    return motion_manager_->isFinished();
}

bool Robot::isMotionBusy() const {
    MutexGuard guard(runtime_mutex_);

    if (!motion_manager_.has_value()) {
        return false;
    }

    return motion_manager_->isBusy();
}

void Robot::cancelMotion() {
    MutexGuard guard(runtime_mutex_);

    if (motion_manager_.has_value()) {
        motion_manager_->cancel();
    }
}

void Robot::waitUntilSettled() {
    const std::uint32_t delay_ms = periodToMilliseconds(config_.motionPeriodSeconds);

    while (!isMotionComplete()) {
        delayMilliseconds(delay_ms);
    }
}

void Robot::tank(double left, double right) {
    MutexGuard guard(runtime_mutex_);

    if (motion_manager_.has_value()) {
        motion_manager_->cancel();
    }

    if (drivetrain_) {
        drivetrain_->tank(left, right);
    }
}

void Robot::arcade(double forward, double turn) {
    MutexGuard guard(runtime_mutex_);

    if (motion_manager_.has_value()) {
        motion_manager_->cancel();
    }

    if (drivetrain_) {
        drivetrain_->arcade(forward, turn);
    }
}

void Robot::configureHardware(
    const DrivetrainConfig& drivetrainConfig,
    const std::vector<int>& leftMotorPorts,
    const std::vector<int>& rightMotorPorts,
    const std::optional<RobotIMUConfig>& imuConfig,
    const std::vector<RobotTrackerConfig>& trackerConfigs,
    const std::vector<RobotDistanceSensorConfig>& distanceSensorConfigs) {

    robot_model_.setDrivetrainConfig(drivetrainConfig);

    sensor_suite_.getTrackerWheels().reserve(trackerConfigs.size());
    sensor_suite_.getDistanceSensors().reserve(distanceSensorConfigs.size());

    if (imuConfig.has_value()) {
        sensor_suite_.setIMU(IMUSensor(imuConfig->port, 0.0));

        IMUSensor* imu_pointer = sensor_suite_.getIMU();
        if (imu_pointer != nullptr) {
            robot_model_.setMountedIMU(
                MountedIMU{imu_pointer, imuConfig->yawOffsetRadians});
        }
    }

    for (const RobotTrackerConfig& trackerConfig : trackerConfigs) {
        sensor_suite_.addTrackerWheel(
            TrackerWheel(
                RotationSensor(trackerConfig.port, trackerConfig.reversed),
                trackerConfig.wheelRadiusInches,
                trackerConfig.gearRatio,
                trackerConfig.axis));

        TrackerWheel* tracker_pointer = nullptr;

        if (!sensor_suite_.getTrackerWheels().empty()) {
            tracker_pointer = &sensor_suite_.getTrackerWheels().back();
        }

        robot_model_.addMountedTrackerWheel(
            MountedTrackerWheel{
                tracker_pointer,
                trackerConfig.positionInRobotFrame});
    }

    for (const RobotDistanceSensorConfig& distanceConfig : distanceSensorConfigs) {
        sensor_suite_.addDistanceSensor(DistanceSensor(distanceConfig.port));

        DistanceSensor* sensor_pointer = nullptr;

        if (!sensor_suite_.getDistanceSensors().empty()) {
            sensor_pointer = &sensor_suite_.getDistanceSensors().back();
        }

        robot_model_.addMountedDistanceSensor(
            MountedDistanceSensor{
                sensor_pointer,
                distanceConfig.poseInRobotFrame,
                distanceConfig.maxRangeInches,
                distanceConfig.beamAngleOffsetRadians});
    }

    drivetrain_ = std::make_unique<TankDrive>(
        MotorGroup(leftMotorPorts),
        MotorGroup(rightMotorPorts),
        drivetrainConfig);
}

std::unique_ptr<StateEstimator> Robot::createStateEstimator() {
    if (config_.localizationMode == LocalizationMode::MCL && field_map_.has_value()) {
        return std::make_unique<MCLEstimator>(
            sensor_suite_,
            robot_model_,
            *field_map_);
    }

    return std::make_unique<OdometryEstimator>(
        sensor_suite_,
        robot_model_);
}

void Robot::localizationTaskEntry(void* robotPointer) {
    Robot* robot = static_cast<Robot*>(robotPointer);

    if (robot != nullptr) {
        robot->runLocalizationLoop();
    }
}

void Robot::motionTaskEntry(void* robotPointer) {
    Robot* robot = static_cast<Robot*>(robotPointer);

    if (robot != nullptr) {
        robot->runMotionLoop();
    }
}

void Robot::runLocalizationLoop() {
    const std::uint32_t delay_ms = periodToMilliseconds(config_.localizationPeriodSeconds);

    while (!shutdown_requested_.load()) {
        if (!runtime_active_.load()) {
            delayMilliseconds(delay_ms);
            continue;
        }

        {
            MutexGuard guard(runtime_mutex_);

            if (state_estimator_) {
                state_estimator_->step(config_.localizationPeriodSeconds);
            }
        }

        delayMilliseconds(delay_ms);
    }
}

void Robot::runMotionLoop() {
    const std::uint32_t delay_ms = periodToMilliseconds(config_.motionPeriodSeconds);

    while (!shutdown_requested_.load()) {
        if (!runtime_active_.load()) {
            delayMilliseconds(delay_ms);
            continue;
        }

        {
            MutexGuard guard(runtime_mutex_);

            if (motion_manager_.has_value() && motion_manager_->isBusy()) {
                motion_manager_->update(config_.motionPeriodSeconds);
            }
        }

        delayMilliseconds(delay_ms);
    }
}

}  // namespace studentlib