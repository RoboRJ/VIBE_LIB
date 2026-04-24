#include "support.hpp"

#include <cmath>
#include <memory>
#include <optional>
#include <vector>

#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

#include "studentlib/math/point.hpp"
#include "studentlib/math/pose.hpp"
#include "studentlib/model/drivetrain_config.hpp"
#include "studentlib/robot/robot_config.hpp"

namespace demo {

namespace {

std::unique_ptr<studentlib::Robot> robot_instance;

const char* motionStatusToString(studentlib::MotionStatus status) {
    switch (status) {
        case studentlib::MotionStatus::IDLE:
            return "IDLE";
        case studentlib::MotionStatus::RUNNING:
            return "RUNNING";
        case studentlib::MotionStatus::SETTLED:
            return "SETTLED";
        case studentlib::MotionStatus::TIMED_OUT:
            return "TIMED_OUT";
        case studentlib::MotionStatus::CANCELED:
            return "CANCELED";
        case studentlib::MotionStatus::ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
    }
}

double applyDeadband(double value, double deadband) {
    if (std::abs(value) <= deadband) {
        return 0.0;
    }

    return value;
}

std::unique_ptr<studentlib::Robot> makeHypotheticalRobot() {
    studentlib::RobotConfig robot_config;
    robot_config.localizationMode = studentlib::LocalizationMode::ODOMETRY;
    robot_config.chassisModel = studentlib::ChassisModel::DIFFERENTIAL;
    robot_config.localizationPeriodSeconds = 0.01;
    robot_config.motionPeriodSeconds = 0.01;

    studentlib::DrivetrainConfig drivetrain_config;
    drivetrain_config.trackWidthInches = 11.5;
    drivetrain_config.wheelRadiusInches = 1.625;
    drivetrain_config.gearRatio = 1.0;
    drivetrain_config.maxVoltageMillivolts = 12000.0;

    const std::vector<int> left_motor_ports{1, 2, 3};

    // Right side reversed. MotorGroup supports negative ports as reversed motors.
    const std::vector<int> right_motor_ports{-4, -5, -6};

    const std::optional<studentlib::RobotIMUConfig> imu_config =
        studentlib::RobotIMUConfig{
            7,
            0.0
        };

    const std::vector<studentlib::RobotTrackerConfig> tracker_configs{
        studentlib::RobotTrackerConfig{
            8,
            false,
            1.0,
            1.0,
            studentlib::TrackerAxis::LONGITUDINAL,
            studentlib::Point2D{0.0, 0.0}
        },
        studentlib::RobotTrackerConfig{
            9,
            false,
            1.0,
            1.0,
            studentlib::TrackerAxis::LATERAL,
            studentlib::Point2D{0.0, 0.0}
        }
    };

    const std::vector<studentlib::RobotDistanceSensorConfig> distance_sensor_configs{};

    return std::make_unique<studentlib::Robot>(
        robot_config,
        drivetrain_config,
        left_motor_ports,
        right_motor_ports,
        imu_config,
        tracker_configs,
        distance_sensor_configs,
        std::nullopt);
}

}  // namespace

studentlib::Robot& robot() {
    if (!robot_instance) {
        robot_instance = makeHypotheticalRobot();
    }

    return *robot_instance;
}

void initializeRobot() {
    studentlib::Robot& r = robot();

    r.initialize();

    // Conservative pause for IMU reset/calibration during first bring-up.
    pros::delay(1500);

    r.start();
    r.setPose(studentlib::Pose2D{0.0, 0.0, 0.0});
}

void printPoseAndStatus(const char* label) {
    studentlib::Robot& r = robot();
    const studentlib::Pose2D pose = r.getPose();

    pros::lcd::print(0, "%s", label);
    pros::lcd::print(1, "x: %.2f in", pose.x);
    pros::lcd::print(2, "y: %.2f in", pose.y);
    pros::lcd::print(3, "theta: %.3f rad", pose.theta);
    pros::lcd::print(4, "status: %s", motionStatusToString(r.getMotionStatus()));
}

double joystickToNormalized(double joystickValue) {
    constexpr double kJoystickMax = 127.0;
    constexpr double kDeadband = 8.0;

    return applyDeadband(joystickValue, kDeadband) / kJoystickMax;
}

}  // namespace demo