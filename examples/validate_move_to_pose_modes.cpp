#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

#include "studentlib/api/builder.hpp"
#include "studentlib/math/point.hpp"
#include "studentlib/math/pose.hpp"
#include "studentlib/motion/motion_params.hpp"
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

void printModeResult(const Robot& robot, const char* modeName) {
    const Pose2D pose = robot.getPose();

    pros::lcd::print(0, "Mode: %s", modeName);
    pros::lcd::print(1, "Status: %s", statusText(robot.getMotionStatus()));
    pros::lcd::print(2, "x %.2f y %.2f", pose.x, pose.y);
    pros::lcd::print(3, "th %.2f", pose.theta);
}

Robot createRobotForMoveToPoseModes() {
    return Builder()
        .withTankDrive({1, 2}, {3, 4}, 12.0, 1.625)
        .withIMU(5)
        .withVerticalTracker(6, 1.0, 1.0, Point2D{0.0, 0.0}, false)
        .withHorizontalTracker(7, 1.0, 1.0, Point2D{0.0, 0.0}, false)
        .withLocalizationMode(LocalizationMode::ODOMETRY)
        .build();
}

MoveToPoseParams baseParams() {
    MoveToPoseParams params;
    params.linear.kP = 0.06;
    params.angular.kP = 0.65;
    params.angular.kD = 0.02;
    params.maxSpeed = 0.35;
    params.minSpeed = 0.08;
    params.settle.positionTolerance = 1.5;
    params.settle.timeoutSeconds = 5.0;
    params.maxFinalAngleError = 0.12;
    return params;
}

}  // namespace

void validateMoveToPoseModes() {
    pros::lcd::initialize();

    auto robot = createRobotForMoveToPoseModes();

    robot.initialize();
    robot.start();

    const Pose2D start{0.0, 0.0, 0.0};
    const Pose2D target{18.0, 8.0, 1.5708};

    MoveToPoseParams pose_regulator = baseParams();
    pose_regulator.pathMode = MoveToPosePathMode::POSE_REGULATOR;

    robot.setPose(start);
    pros::lcd::print(0, "Testing POSE_REGULATOR");
    pros::delay(1000);

    // Expected: direct regulation toward the target pose.
    robot.moveToPose(target, pose_regulator);
    printModeResult(robot, "POSE_REGULATOR");
    pros::delay(1500);

    MoveToPoseParams straight_then_turn = baseParams();
    straight_then_turn.pathMode = MoveToPosePathMode::STRAIGHT_THEN_TURN;

    robot.setPose(start);
    pros::lcd::print(0, "Testing STRAIGHT_THEN_TURN");
    pros::delay(1000);

    // Expected: distinct drive-to-position stage, then final turn stage.
    // Prefer this mode when a predictable staged motion is easier to tune.
    robot.moveToPose(target, straight_then_turn);
    printModeResult(robot, "STRAIGHT_THEN_TURN");
    pros::delay(1500);

    MoveToPoseParams bezier = baseParams();
    bezier.pathMode = MoveToPosePathMode::QUADRATIC_BEZIER;
    bezier.quadraticBezier.leadFactor = 0.5;
    bezier.quadraticBezier.lookaheadT = 0.08;
    bezier.quadraticBezier.finalHeadingSwitchDistance = 6.0;

    robot.setPose(start);
    pros::lcd::print(0, "Testing QUADRATIC_BEZIER");
    pros::delay(1000);

    // Expected: smoother curved approach.
    // If the curve is too aggressive, reduce leadFactor.
    // If it feels sluggish along the curve, adjust lookaheadT carefully.
    robot.moveToPose(target, bezier);
    printModeResult(robot, "QUADRATIC_BEZIER");
    pros::delay(1500);

    while (true) {
        printModeResult(robot, "All modes tested");
        pros::delay(100);
    }
}