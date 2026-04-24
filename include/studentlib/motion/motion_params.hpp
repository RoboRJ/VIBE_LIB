#pragma once

#include "studentlib/control/pid_config.hpp"
#include "studentlib/control/settle_config.hpp"

namespace studentlib {

enum class MoveToPosePathMode {
    POSE_REGULATOR,
    STRAIGHT_THEN_TURN,
    QUADRATIC_BEZIER
};

struct QuadraticBezierPathParams {
    double leadFactor{0.4};
    double searchWindowT{0.2};
    double lookaheadT{0.05};
    double finalHeadingSwitchDistance{6.0};
};

struct TurnToHeadingParams {
    PIDConfig angular{};
    double maxSpeed{1.0};
    double minSpeed{0.0};
    SettleConfig settle{};
};

struct TurnToPointParams {
    PIDConfig angular{};
    double maxSpeed{1.0};
    double minSpeed{0.0};
    SettleConfig settle{};
};

struct DriveToPointParams {
    PIDConfig linear{};
    PIDConfig angular{};
    double maxSpeed{1.0};
    double minSpeed{0.0};
    // reverse = true means the robot is allowed to drive backward to the point.
    double reverse{false};
    SettleConfig settle{};
};

struct MoveToPoseParams {
    PIDConfig linear{};
    PIDConfig angular{};
    double maxSpeed{1.0};
    double minSpeed{0.0};
    // reverse = true means the approach motion may drive backward toward the target point.
    bool reverse{false};
    // approachHeadingWeight is a normalized [0, 1] style preference for aiming
    // toward the target point while still blending toward the final target heading.
    double approachHeadingWeight{1.0};
    double maxFinalAngleError{0.05};
    SettleConfig settle{};

    MoveToPosePathMode pathMode{MoveToPosePathMode::POSE_REGULATOR};
    QuadraticBezierPathParams quadraticBezier{};
};

}  // namespace studentlib