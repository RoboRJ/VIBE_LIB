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

void printPoseAndStatus(const Robot& robot, int labelLine, const char* label) {
    const Pose2D pose = robot.getPose();

    pros::lcd::print(labelLine, "%s", label);
    pros::lcd::print(labelLine + 1, "x %.1f y %.1f", pose.x, pose.y);
    pros::lcd::print(labelLine + 2, "th %.2f %s",
                     pose.theta,
                     statusText(robot.getMotionStatus()));
}

Robot createRobotForLocalizationSquare() {
    return Builder()
        .withTankDrive({1, 2}, {3, 4}, 12.0, 1.625)
        .withIMU(5)
        .withVerticalTracker(6, 1.0, 1.0, Point2D{0.0, 0.0}, false)
        .withHorizontalTracker(7, 1.0, 1.0, Point2D{0.0, 0.0}, false)
        .withLocalizationMode(LocalizationMode::ODOMETRY)
        .build();

    // To compare with MCL, use the same robot configuration but add:
    // - distance sensors
    // - a FieldMap
    // - withLocalizationMode(LocalizationMode::MCL)
}

}  // namespace

void validateLocalizationSquare() {
    pros::lcd::initialize();

    auto robot = createRobotForLocalizationSquare();

    robot.initialize();
    robot.start();
    robot.setPose(Pose2D{0.0, 0.0, 0.0});

    MoveToPoseParams params;
    params.maxSpeed = 0.35;
    params.minSpeed = 0.08;
    params.settle.positionTolerance = 1.5;
    params.settle.timeoutSeconds = 4.0;
    params.maxFinalAngleError = 0.15;
    params.pathMode = MoveToPosePathMode::STRAIGHT_THEN_TURN;

    // Small axis-aligned square.
    // Good enough for first bring-up: pose should be in the rough expected quadrant
    // after each leg and should return near the start after the final leg.
    // Large drift usually means tracker sign, wheel radius, gear ratio, or heading sign is wrong.

    robot.moveToPose(Pose2D{12.0, 0.0, 0.0}, params);
    printPoseAndStatus(robot, 0, "After +X");
    pros::delay(1000);

    robot.moveToPose(Pose2D{12.0, 12.0, 1.5708}, params);
    printPoseAndStatus(robot, 0, "After +Y");
    pros::delay(1000);

    robot.moveToPose(Pose2D{0.0, 12.0, 3.1416}, params);
    printPoseAndStatus(robot, 0, "After -X");
    pros::delay(1000);

    robot.moveToPose(Pose2D{0.0, 0.0, 0.0}, params);
    printPoseAndStatus(robot, 0, "Back near start");
    pros::delay(1000);

    while (true) {
        printPoseAndStatus(robot, 0, "Final pose");
        pros::delay(100);
    }
}