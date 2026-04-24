#include "studentlib/api/builder.hpp"
#include "studentlib/math/point.hpp"
#include "studentlib/math/pose.hpp"
#include "studentlib/motion/motion_params.hpp"

using namespace studentlib;

void tuningExamples() {
    auto robot = Builder()
        .withTankDrive({1, 2}, {3, 4}, 12.0, 1.625)
        .withIMU(5)
        .withVerticalTracker(6, 1.0, 1.0, Point2D{0.0, 0.0}, false)
        .withHorizontalTracker(7, 1.0, 1.0, Point2D{0.0, 0.0}, false)
        .build();

    robot.initialize();
    robot.setPose(Pose2D{0.0, 0.0, 0.0});

    MoveToPoseParams move_params;

    // Linear PID: raise kP to respond harder to distance error.
    move_params.linear.kP = 0.08;
    move_params.linear.kI = 0.0;
    move_params.linear.kD = 0.002;

    // Angular PID: helps the robot point where it should go.
    move_params.angular.kP = 0.9;
    move_params.angular.kI = 0.0;
    move_params.angular.kD = 0.02;

    // minSpeed helps overcome stiction when the command is small but nonzero.
    move_params.minSpeed = 0.10;

    // maxSpeed caps overall aggressiveness.
    move_params.maxSpeed = 0.75;

    // Longer timeout gives the robot more time before the motion ends as TIMED_OUT.
    move_params.settle.timeoutSeconds = 3.0;

    // Tighter angle error means the final heading must be closer before settling.
    move_params.maxFinalAngleError = 0.04;

    robot.moveToPose(Pose2D{36.0, 12.0, 1.57}, move_params);

    MoveToPoseParams bezier_params = move_params;
    bezier_params.pathMode = MoveToPosePathMode::QUADRATIC_BEZIER;

    // leadFactor changes how strongly the curve approaches from behind the target heading.
    bezier_params.quadraticBezier.leadFactor = 0.6;

    // More lookahead usually makes the path reference less "stuck" near the robot.
    bezier_params.quadraticBezier.lookaheadT = 0.08;

    robot.moveToPose(Pose2D{60.0, 24.0, 1.57}, bezier_params);

    TurnToHeadingParams turn_params;
    turn_params.angular.kP = 1.0;
    turn_params.angular.kD = 0.03;
    turn_params.maxSpeed = 0.6;
    turn_params.minSpeed = 0.08;
    turn_params.settle.timeoutSeconds = 1.5;

    robot.turnToHeading(3.14, turn_params);
}