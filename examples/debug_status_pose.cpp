#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

#include "studentlib/api/builder.hpp"
#include "studentlib/math/point.hpp"
#include "studentlib/math/pose.hpp"
#include "studentlib/motion/motion_status.hpp"

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

}  // namespace

void debugExample() {
    pros::lcd::initialize();

    auto robot = Builder()
        .withTankDrive({1, 2}, {3, 4}, 12.0, 1.625)
        .withIMU(5)
        .withVerticalTracker(6, 1.0, 1.0, Point2D{0.0, 0.0}, false)
        .withHorizontalTracker(7, 1.0, 1.0, Point2D{0.0, 0.0}, false)
        .build();

    robot.initialize();
    robot.setPose(Pose2D{0.0, 0.0, 0.0});
    robot.startMoveToPose(Pose2D{24.0, 24.0, 1.57});

    while (true) {
        const Pose2D pose = robot.getPose();
        const MotionStatus status = robot.getMotionStatus();

        pros::lcd::print(0, "x: %.2f", pose.x);
        pros::lcd::print(1, "y: %.2f", pose.y);
        pros::lcd::print(2, "th: %.2f", pose.theta);
        pros::lcd::print(3, "status: %s", motionStatusToString(status));
        pros::lcd::print(4, "busy: %d", robot.isMotionBusy() ? 1 : 0);
        pros::lcd::print(5, "done: %d", robot.isMotionComplete() ? 1 : 0);

        pros::delay(50);
    }
}