#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

#include "studentlib/api/builder.hpp"
#include "studentlib/math/point.hpp"
#include "studentlib/math/pose.hpp"
#include "studentlib/motion/motion_status.hpp"
#include "studentlib/robot/robot_config.hpp"

using namespace studentlib;

namespace {

const char* statusText(MotionStatus status) {
    switch (status) {
        case MotionStatus::IDLE:
            return "IDLE";
        case MotionStatus::RUNNING:
            return "RUNNING";
        case MotionStatus::SETTLED:
            return "SETTLED";
        case MotionStatus::TIMED_OUT:
            return "TIMED_OUT";
        case MotionStatus::CANCELED:
            return "CANCELED";
        case MotionStatus::ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
    }
}

void printStatusAndPose(const Robot& robot, const char* label) {
    const Pose2D pose = robot.getPose();

    pros::lcd::print(0, "%s", label);
    pros::lcd::print(1, "status: %s", statusText(robot.getMotionStatus()));
    pros::lcd::print(2, "x %.2f y %.2f", pose.x, pose.y);
    pros::lcd::print(3, "theta %.2f", pose.theta);
}

Robot createRobotForTimeoutCancelValidation() {
    return Builder()
        .withTankDrive({1, 2}, {3, 4}, 12.0, 1.625)
        .withIMU(5)
        .withVerticalTracker(6, 1.0, 1.0, Point2D{0.0, 0.0}, false)
        .withHorizontalTracker(7, 1.0, 1.0, Point2D{0.0, 0.0}, false)
        .withLocalizationMode(LocalizationMode::ODOMETRY)
        .build();
}

}  // namespace

void validateTimeoutAndCancel() {
    pros::lcd::initialize();

    auto robot = createRobotForTimeoutCancelValidation();

    robot.initialize();
    robot.start();
    robot.setPose(Pose2D{0.0, 0.0, 0.0});

    MoveToPoseParams cancel_params;
    cancel_params.linear.kP = 0.06;
    cancel_params.angular.kP = 0.65;
    cancel_params.maxSpeed = 0.30;
    cancel_params.minSpeed = 0.08;
    cancel_params.settle.positionTolerance = 1.5;
    cancel_params.settle.timeoutSeconds = 5.0;
    cancel_params.maxFinalAngleError = 0.12;

    robot.startMoveToPose(Pose2D{24.0, 0.0, 0.0}, cancel_params);
    pros::delay(400);

    robot.cancelMotion();
    printStatusAndPose(robot, "After cancel");

    // Expected status: CANCELED.
    // If the robot keeps moving after cancel, check drivetrain stop behavior.
    pros::delay(1500);

    MoveToPoseParams timeout_params = cancel_params;

    // Deliberately short timeout.
    // Keep target small and speed conservative so this remains safe.
    timeout_params.settle.timeoutSeconds = 0.05;

    robot.setPose(Pose2D{0.0, 0.0, 0.0});
    robot.moveToPose(Pose2D{18.0, 0.0, 0.0}, timeout_params);

    printStatusAndPose(robot, "After timeout");

    // Expected status: TIMED_OUT.
    // If a normal command times out unexpectedly, likely causes are:
    // - timeoutSeconds too short
    // - localization sign error
    // - maxSpeed too low
    // - minSpeed too low to move the robot
    // - settling tolerance too strict

    while (true) {
        pros::delay(100);
    }
}