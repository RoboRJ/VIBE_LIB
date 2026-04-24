#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "pros/rtos.hpp"
#include "studentlib/drivetrain/drivetrain.hpp"
#include "studentlib/estimation/state_estimator.hpp"
#include "studentlib/hardware/sensor_suite.hpp"
#include "studentlib/math/point.hpp"
#include "studentlib/math/pose.hpp"
#include "studentlib/model/drivetrain_config.hpp"
#include "studentlib/model/field_map.hpp"
#include "studentlib/model/robot_model.hpp"
#include "studentlib/motion/motion_manager.hpp"
#include "studentlib/motion/motion_params.hpp"
#include "studentlib/motion/motion_status.hpp"
#include "studentlib/robot/robot_config.hpp"

namespace studentlib {

struct RobotIMUConfig {
    std::int8_t port{0};
    double yawOffsetRadians{0.0};
};

struct RobotTrackerConfig {
    std::int8_t port{0};
    bool reversed{false};
    double wheelRadiusInches{0.0};
    double gearRatio{1.0};
    TrackerAxis axis{TrackerAxis::LONGITUDINAL};
    Point2D positionInRobotFrame{};
};

struct RobotDistanceSensorConfig {
    std::uint8_t port{0};
    Pose2D poseInRobotFrame{};
    double maxRangeInches{0.0};
    double beamAngleOffsetRadians{0.0};
};

class Robot {
public:
    explicit Robot(
        const RobotConfig& robotConfig,
        const DrivetrainConfig& drivetrainConfig,
        const std::vector<int>& leftMotorPorts,
        const std::vector<int>& rightMotorPorts,
        const std::optional<RobotIMUConfig>& imuConfig = std::nullopt,
        const std::vector<RobotTrackerConfig>& trackerConfigs = {},
        const std::vector<RobotDistanceSensorConfig>& distanceSensorConfigs = {},
        const std::optional<FieldMap>& fieldMap = std::nullopt);

    ~Robot();

    void initialize();
    void start();
    void stop();

    Pose2D getPose() const;
    void setPose(const Pose2D& pose);
    void resetPose();

    void moveToPose(const Pose2D& target, const MoveToPoseParams& params = {});
    void turnToPoint(const Point2D& target, const TurnToPointParams& params = {});
    void turnToHeading(double headingRadians, const TurnToHeadingParams& params = {});
    void driveToPoint(const Point2D& target, const DriveToPointParams& params = {});

    void startMoveToPose(const Pose2D& target, const MoveToPoseParams& params = {});
    void startTurnToPoint(const Point2D& target, const TurnToPointParams& params = {});
    void startTurnToHeading(double headingRadians, const TurnToHeadingParams& params = {});
    void startDriveToPoint(const Point2D& target, const DriveToPointParams& params = {});

    MotionStatus getMotionStatus() const;
    bool isMotionComplete() const;
    bool isMotionBusy() const;
    void cancelMotion();
    void waitUntilSettled();

    void tank(double left, double right);
    void arcade(double forward, double turn);

private:
    RobotConfig config_{};

    SensorSuite sensor_suite_{};
    RobotModel robot_model_{};
    std::optional<FieldMap> field_map_{};

    std::unique_ptr<Drivetrain> drivetrain_{};
    std::unique_ptr<StateEstimator> state_estimator_{};
    std::optional<MotionManager> motion_manager_{};

    mutable pros::Mutex runtime_mutex_;
    std::unique_ptr<pros::Task> localization_task_{};
    std::unique_ptr<pros::Task> motion_task_{};

    std::atomic<bool> initialized_{false};
    std::atomic<bool> runtime_active_{false};
    std::atomic<bool> shutdown_requested_{false};

    void configureHardware(
        const DrivetrainConfig& drivetrainConfig,
        const std::vector<int>& leftMotorPorts,
        const std::vector<int>& rightMotorPorts,
        const std::optional<RobotIMUConfig>& imuConfig,
        const std::vector<RobotTrackerConfig>& trackerConfigs,
        const std::vector<RobotDistanceSensorConfig>& distanceSensorConfigs);

    std::unique_ptr<StateEstimator> createStateEstimator();

    static void localizationTaskEntry(void* robotPointer);
    static void motionTaskEntry(void* robotPointer);

    void runLocalizationLoop();
    void runMotionLoop();
};

}  // namespace studentlib