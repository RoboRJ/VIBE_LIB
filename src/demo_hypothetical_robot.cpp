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

const char* motionStatusToString(MotionStatus status) {
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

void printPoseAndStatus(const Robot& robot, const char* label) {
    const Pose2D pose = robot.getPose();

    pros::lcd::print(0, "%s", label);
    pros::lcd::print(1, "x: %.2f in", pose.x);
    pros::lcd::print(2, "y: %.2f in", pose.y);
    pros::lcd::print(3, "theta: %.3f rad", pose.theta);
    pros::lcd::print(4, "status: %s", motionStatusToString(robot.getMotionStatus()));
}

Robot createHypotheticalRobot() {
    // Right-side drive motors are reversed in the hypothetical configuration.
    // The current Builder API does not expose separate reversed flags, so this
    // example uses negative motor ports, which MotorGroup treats as reversed.
    return Builder()
        .withTankDrive(
            {1, 2, 3},       // left drive motors
            {-4, -5, -6},    // right drive motors, reversed
            11.5,            // track width in inches
            1.625,           // drive wheel radius in inches
            1.0,             // drive gear ratio
            12000.0)         // max voltage in millivolts
        .withIMU(
            7,               // IMU port
            0.0)             // yaw offset in radians
        .withVerticalTracker(
            8,               // vertical tracker rotation sensor port
            1.0,             // tracker wheel radius in inches
            1.0,             // gear ratio
            Point2D{0.0, 0.0},
            false)           // forward push should increase distance
        .withHorizontalTracker(
            9,               // horizontal tracker rotation sensor port
            1.0,             // tracker wheel radius in inches
            1.0,             // gear ratio
            Point2D{0.0, 0.0},
            false)           // leftward push should increase distance
        .withLocalizationMode(LocalizationMode::ODOMETRY)
        .withChassisModel(ChassisModel::DIFFERENTIAL)
        .build();
}

TurnToHeadingParams makeConservativeTurnParams() {
    TurnToHeadingParams params;

    params.angular.kP = 0.6;
    params.angular.kI = 0.0;
    params.angular.kD = 0.02;

    params.maxSpeed = 0.30;
    params.minSpeed = 0.06;

    params.settle.positionTolerance = 0.08;
    params.settle.settleTimeSeconds = 0.20;
    params.settle.timeoutSeconds = 2.0;

    return params;
}

DriveToPointParams makeConservativeDriveParams() {
    DriveToPointParams params;

    params.linear.kP = 0.06;
    params.linear.kI = 0.0;
    params.linear.kD = 0.002;

    params.angular.kP = 0.6;
    params.angular.kI = 0.0;
    params.angular.kD = 0.02;

    params.maxSpeed = 0.35;
    params.minSpeed = 0.08;
    params.reverse = false;

    params.settle.positionTolerance = 1.0;
    params.settle.settleTimeSeconds = 0.20;
    params.settle.timeoutSeconds = 2.5;

    return params;
}

MoveToPoseParams makeBaseMoveToPoseParams() {
    MoveToPoseParams params;

    params.linear.kP = 0.06;
    params.linear.kI = 0.0;
    params.linear.kD = 0.002;

    params.angular.kP = 0.6;
    params.angular.kI = 0.0;
    params.angular.kD = 0.02;

    params.maxSpeed = 0.35;
    params.minSpeed = 0.08;
    params.reverse = false;

    params.maxFinalAngleError = 0.12;

    params.settle.positionTolerance = 1.0;
    params.settle.settleTimeSeconds = 0.20;
    params.settle.timeoutSeconds = 2.5;

    params.quadraticBezier.leadFactor = 0.40;
    params.quadraticBezier.searchWindowT = 0.20;
    params.quadraticBezier.lookaheadT = 0.05;
    params.quadraticBezier.finalHeadingSwitchDistance = 6.0;

    return params;
}

void pauseAndPrint(const Robot& robot, const char* label) {
    printPoseAndStatus(robot, label);
    pros::delay(1000);
}

}  // namespace

void runHypotheticalRobotDemo() {
    pros::lcd::initialize();

    auto robot = createHypotheticalRobot();

    robot.initialize();
    robot.start();

    robot.setPose(Pose2D{0.0, 0.0, 0.0});
    pauseAndPrint(robot, "Pose reset");

    // Step 1: validate heading control.
    // Expected: small counterclockwise turn to about 0.5 rad.
    // If the robot turns the wrong way, check right/left motor reversal and IMU sign.
    robot.turnToHeading(0.5, makeConservativeTurnParams());
    pauseAndPrint(robot, "After turn");

    // Step 2: validate simple point driving.
    // Expected: short forward-ish movement toward x = 8 inches.
    // If x decreases, check vertical tracker sign.
    robot.driveToPoint(Point2D{8.0, 0.0}, makeConservativeDriveParams());
    pauseAndPrint(robot, "After drive");

    // Step 3: validate staged point-to-pose behavior.
    // Expected: drive to the position first, then turn to the final heading.
    MoveToPoseParams straight_then_turn = makeBaseMoveToPoseParams();
    straight_then_turn.pathMode = MoveToPosePathMode::STRAIGHT_THEN_TURN;

    robot.moveToPose(Pose2D{12.0, 6.0, 0.0}, straight_then_turn);
    pauseAndPrint(robot, "Straight then turn");

    // Step 4: validate quadratic Bézier reference behavior.
    // Expected: smoother curved approach into the final pose.
    // If the curve is too aggressive, reduce leadFactor.
    MoveToPoseParams bezier = makeBaseMoveToPoseParams();
    bezier.pathMode = MoveToPosePathMode::QUADRATIC_BEZIER;
    bezier.quadraticBezier.leadFactor = 0.40;
    bezier.quadraticBezier.lookaheadT = 0.05;

    robot.moveToPose(Pose2D{18.0, 10.0, 1.5708}, bezier);
    pauseAndPrint(robot, "Bezier mode");

    while (true) {
        printPoseAndStatus(robot, "Demo complete");
        pros::delay(100);
    }
}

// Example PROS competition hook.
// In a real project, either call this from autonomous()
// or copy the body of runHypotheticalRobotDemo() into your own autonomous routine.
// uncomment this to run it instead:
// void autonomous() {
//     runHypotheticalRobotDemo();
// }