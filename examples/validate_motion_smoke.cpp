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

void printResult(const Robot& robot, const char* label) {
    const Pose2D pose = robot.getPose();

    pros::lcd::print(0, "%s", label);
    pros::lcd::print(1, "status: %s", statusText(robot.getMotionStatus()));
    pros::lcd::print(2, "x: %.2f", pose.x);
    pros::lcd::print(3, "y: %.2f", pose.y);
    pros::lcd::print(4, "theta: %.2f", pose.theta);
}

Robot createRobotForMotionSmoke() {
    return Builder()
        .withTankDrive({1, 2}, {3, 4}, 12.0, 1.625)
        .withIMU(5)
        .withVerticalTracker(6, 1.0, 1.0, Point2D{0.0, 0.0}, false)
        .withHorizontalTracker(7, 1.0, 1.0, Point2D{0.0, 0.0}, false)
        .withLocalizationMode(LocalizationMode::ODOMETRY)
        .build();
}

}  // namespace

void validateMotionSmoke() {
    pros::lcd::initialize();

    auto robot = createRobotForMotionSmoke();

    robot.initialize();
    robot.start();
    robot.setPose(Pose2D{0.0, 0.0, 0.0});

    TurnToHeadingParams turn_params;
    turn_params.angular.kP = 0.6;
    turn_params.angular.kD = 0.02;
    turn_params.maxSpeed = 0.35;
    turn_params.minSpeed = 0.06;
    turn_params.settle.positionTolerance = 0.08;
    turn_params.settle.timeoutSeconds = 2.0;

    DriveToPointParams drive_params;
    drive_params.linear.kP = 0.06;
    drive_params.angular.kP = 0.6;
    drive_params.angular.kD = 0.02;
    drive_params.maxSpeed = 0.35;
    drive_params.minSpeed = 0.08;
    drive_params.settle.positionTolerance = 1.5;
    drive_params.settle.timeoutSeconds = 3.0;

    MoveToPoseParams move_params;
    move_params.linear.kP = 0.06;
    move_params.angular.kP = 0.6;
    move_params.angular.kD = 0.02;
    move_params.maxSpeed = 0.35;
    move_params.minSpeed = 0.08;
    move_params.settle.positionTolerance = 1.5;
    move_params.settle.timeoutSeconds = 4.0;
    move_params.maxFinalAngleError = 0.12;

    // Test 1: small turn.
    // If it turns the wrong way, check drivetrain side signs and IMU sign.
    robot.turnToHeading(0.7854, turn_params);
    printResult(robot, "turnToHeading");
    pros::delay(1000);

    // Test 2: short drive.
    // If it overshoots badly, lower kP or maxSpeed.
    // If it never settles, loosen positionTolerance or check localization noise.
    robot.driveToPoint(Point2D{8.0, 0.0}, drive_params);
    printResult(robot, "driveToPoint");
    pros::delay(1000);

    // Test 3: short point-to-pose.
    // If timeout happens immediately, check timeoutSeconds and loop start behavior.
    robot.moveToPose(Pose2D{12.0, 6.0, 0.0}, move_params);
    printResult(robot, "moveToPose");
    pros::delay(1000);

    while (true) {
        printResult(robot, "smoke done");
        pros::delay(100);
    }
}