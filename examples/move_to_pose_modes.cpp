#include "studentlib/api/builder.hpp"
#include "studentlib/math/point.hpp"
#include "studentlib/math/pose.hpp"
#include "studentlib/motion/motion_params.hpp"

using namespace studentlib;

void demonstrateMoveToPoseModes() {
    auto robot = Builder()
        .withTankDrive({1, 2}, {3, 4}, 12.0, 1.625)
        .withIMU(5)
        .withVerticalTracker(6, 1.0, 1.0, Point2D{0.0, 0.0}, false)
        .withHorizontalTracker(7, 1.0, 1.0, Point2D{0.0, 0.0}, false)
        .build();

    robot.initialize();
    robot.setPose(Pose2D{0.0, 0.0, 0.0});

    MoveToPoseParams regulator_params;
    regulator_params.pathMode = MoveToPosePathMode::POSE_REGULATOR;
    regulator_params.maxSpeed = 0.8;
    regulator_params.minSpeed = 0.12;
    regulator_params.maxFinalAngleError = 0.08;

    // Good default when you want a direct point-to-pose regulator.
    robot.moveToPose(Pose2D{24.0, 12.0, 1.57}, regulator_params);

    MoveToPoseParams straight_then_turn_params;
    straight_then_turn_params.pathMode = MoveToPosePathMode::STRAIGHT_THEN_TURN;
    straight_then_turn_params.maxSpeed = 0.7;
    straight_then_turn_params.minSpeed = 0.10;
    straight_then_turn_params.maxFinalAngleError = 0.05;

    // Useful when you want a simpler, more predictable "arrive first, rotate second" behavior.
    robot.moveToPose(Pose2D{36.0, 12.0, 3.14}, straight_then_turn_params);

    MoveToPoseParams bezier_params;
    bezier_params.pathMode = MoveToPosePathMode::QUADRATIC_BEZIER;
    bezier_params.maxSpeed = 0.9;
    bezier_params.minSpeed = 0.10;
    bezier_params.maxFinalAngleError = 0.06;
    bezier_params.quadraticBezier.leadFactor = 0.5;
    bezier_params.quadraticBezier.lookaheadT = 0.08;
    bezier_params.quadraticBezier.finalHeadingSwitchDistance = 8.0;

    // Useful when you want a smoother approach shape into the target pose.
    // STRAIGHT_THEN_TURN is often simpler to tune if the curved approach feels unnecessary.
    robot.moveToPose(Pose2D{48.0, 24.0, 1.57}, bezier_params);
}