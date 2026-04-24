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

Robot createRobotForNonblockingValidation() {
    return Builder()
        .withTankDrive({1, 2}, {3, 4}, 12.0, 1.625)
        .withIMU(5)
        .withVerticalTracker(6, 1.0, 1.0, Point2D{0.0, 0.0}, false)
        .withHorizontalTracker(7, 1.0, 1.0, Point2D{0.0, 0.0}, false)
        .withLocalizationMode(LocalizationMode::ODOMETRY)
        .build();
}

}  // namespace

void validateNonblockingMotion() {
    pros::lcd::initialize();

    auto robot = createRobotForNonblockingValidation();

    robot.initialize();
    robot.start();
    robot.setPose(Pose2D{0.0, 0.0, 0.0});

    MoveToPoseParams params;
    params.linear.kP = 0.06;
    params.angular.kP = 0.65;
    params.angular.kD = 0.02;
    params.maxSpeed = 0.35;
    params.minSpeed = 0.08;
    params.settle.positionTolerance = 1.5;
    params.settle.timeoutSeconds = 5.0;
    params.maxFinalAngleError = 0.12;

    robot.startMoveToPose(Pose2D{18.0, 0.0, 0.0}, params);

    bool mechanism_state = false;
    int loop_count = 0;

    // Expected: startMoveToPose(...) returns immediately.
    // This loop simulates mechanism work while the chassis is moving.
    while (!robot.isMotionComplete()) {
        if ((loop_count % 25) == 0) {
            mechanism_state = !mechanism_state;

            // Replace this with real mechanism code, such as:
            // intakeMotor.move(127);
            // piston.set_value(true);
        }

        const Pose2D pose = robot.getPose();
        const MotionStatus status = robot.getMotionStatus();

        pros::lcd::print(0, "nonblocking active");
        pros::lcd::print(1, "status: %s", statusText(status));
        pros::lcd::print(2, "x %.2f y %.2f", pose.x, pose.y);
        pros::lcd::print(3, "mechanism: %d", mechanism_state ? 1 : 0);

        if (status == MotionStatus::TIMED_OUT || status == MotionStatus::ERROR) {
            break;
        }

        ++loop_count;
        pros::delay(20);
    }

    const Pose2D final_pose = robot.getPose();

    pros::lcd::print(0, "nonblocking done");
    pros::lcd::print(1, "status: %s", statusText(robot.getMotionStatus()));
    pros::lcd::print(2, "x %.2f y %.2f", final_pose.x, final_pose.y);
    pros::lcd::print(3, "theta %.2f", final_pose.theta);

    while (true) {
        pros::delay(100);
    }
}