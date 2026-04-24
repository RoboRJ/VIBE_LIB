#include "pros/motors.hpp"
#include "pros/rtos.hpp"

#include "studentlib/api/builder.hpp"
#include "studentlib/math/point.hpp"
#include "studentlib/math/pose.hpp"
#include "studentlib/motion/motion_status.hpp"

using namespace studentlib;

void autonomous() {
    auto robot = Builder()
        .withTankDrive({1, 2}, {3, 4}, 12.0, 1.625)
        .withIMU(5)
        .withVerticalTracker(6, 1.0, 1.0, Point2D{0.0, 0.0}, false)
        .withHorizontalTracker(7, 1.0, 1.0, Point2D{0.0, 0.0}, false)
        .build();

    pros::Motor intake_motor(10);  // example mechanism motor

    robot.initialize();
    robot.setPose(Pose2D{0.0, 0.0, 0.0});

    robot.startMoveToPose(Pose2D{48.0, 0.0, 0.0});

    // While the chassis is moving, run another mechanism.
    while (!robot.isMotionComplete()) {
        intake_motor.move(127);

        // You can also react to status if needed.
        MotionStatus status = robot.getMotionStatus();
        if (status == MotionStatus::TIMED_OUT) {
            break;
        }

        pros::delay(10);
    }

    intake_motor.move(0);

    // Continue the sequence after the move finishes.
    robot.turnToPoint(Point2D{48.0, 24.0});
}