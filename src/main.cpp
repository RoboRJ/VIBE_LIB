#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

#include "support.hpp"

#include "studentlib/math/point.hpp"
#include "studentlib/math/pose.hpp"
#include "studentlib/motion/motion_params.hpp"

using namespace studentlib;

namespace {

TurnToHeadingParams turnParams() {
    TurnToHeadingParams params;

    params.angular.kP = 0.6;
    params.angular.kD = 0.02;

    params.maxSpeed = 0.30;
    params.minSpeed = 0.06;

    params.settle.positionTolerance = 0.08;
    params.settle.settleTimeSeconds = 0.20;
    params.settle.timeoutSeconds = 2.0;

    return params;
}

DriveToPointParams driveParams() {
    DriveToPointParams params;

    params.linear.kP = 0.06;
    params.linear.kD = 0.002;

    params.angular.kP = 0.6;
    params.angular.kD = 0.02;

    params.maxSpeed = 0.35;
    params.minSpeed = 0.08;

    params.settle.positionTolerance = 1.0;
    params.settle.settleTimeSeconds = 0.20;
    params.settle.timeoutSeconds = 2.5;

    return params;
}

MoveToPoseParams moveParams(MoveToPosePathMode mode) {
    MoveToPoseParams params;

    params.linear.kP = 0.06;
    params.linear.kD = 0.002;

    params.angular.kP = 0.6;
    params.angular.kD = 0.02;

    params.maxSpeed = 0.35;
    params.minSpeed = 0.08;

    params.maxFinalAngleError = 0.12;

    params.settle.positionTolerance = 1.0;
    params.settle.settleTimeSeconds = 0.20;
    params.settle.timeoutSeconds = 2.5;

    params.pathMode = mode;

    params.quadraticBezier.leadFactor = 0.40;
    params.quadraticBezier.searchWindowT = 0.20;
    params.quadraticBezier.lookaheadT = 0.05;
    params.quadraticBezier.finalHeadingSwitchDistance = 6.0;

    return params;
}

}  // namespace

void initialize() {
    pros::lcd::initialize();

    demo::initializeRobot();
    demo::printPoseAndStatus("initialized");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    Robot& robot = demo::robot();

    robot.setPose(Pose2D{0.0, 0.0, 0.0});
    demo::printPoseAndStatus("auton start");
    pros::delay(750);

    // Validate heading control.
    robot.turnToHeading(0.5, turnParams());
    demo::printPoseAndStatus("after turn");
    pros::delay(750);

    // Validate short +x motion.
    robot.driveToPoint(Point2D{8.0, 0.0}, driveParams());
    demo::printPoseAndStatus("after drive");
    pros::delay(750);

    // Validate staged position-then-heading behavior.
    robot.moveToPose(
        Pose2D{12.0, 6.0, 0.0},
        moveParams(MoveToPosePathMode::STRAIGHT_THEN_TURN));

    demo::printPoseAndStatus("after straight");
    pros::delay(750);

    // Validate curved approach reference.
    robot.moveToPose(
        Pose2D{18.0, 10.0, 1.5708},
        moveParams(MoveToPosePathMode::QUADRATIC_BEZIER));

    demo::printPoseAndStatus("after bezier");
}

void opcontrol() {
    Robot& robot = demo::robot();
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    while (true) {
        const double forward = demo::joystickToNormalized(
            master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));

        const double turn = demo::joystickToNormalized(
            master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        robot.arcade(forward, turn);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            robot.setPose(Pose2D{0.0, 0.0, 0.0});
        }

        demo::printPoseAndStatus("opcontrol");

        pros::delay(20);
    }
}