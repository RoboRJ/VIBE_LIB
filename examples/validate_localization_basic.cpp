#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

#include "studentlib/api/builder.hpp"
#include "studentlib/math/point.hpp"
#include "studentlib/math/pose.hpp"
#include "studentlib/robot/robot_config.hpp"

using namespace studentlib;

namespace {

void printPose(const Pose2D& pose) {
    pros::lcd::print(0, "x: %.2f in", pose.x);
    pros::lcd::print(1, "y: %.2f in", pose.y);
    pros::lcd::print(2, "theta: %.3f rad", pose.theta);
}

Robot createRobotForLocalizationBasic() {
    return Builder()
        .withTankDrive(
            {1, 2},     // example left motor ports
            {3, 4},     // example right motor ports
            12.0,       // track width in inches
            1.625)      // drive wheel radius in inches
        .withIMU(5)
        .withVerticalTracker(
            6,
            1.0,
            1.0,
            Point2D{0.0, 0.0},
            false)
        .withHorizontalTracker(
            7,
            1.0,
            1.0,
            Point2D{0.0, 0.0},
            false)
        .withLocalizationMode(LocalizationMode::ODOMETRY)
        .build();
}

}  // namespace

void validateLocalizationBasic() {
    pros::lcd::initialize();

    auto robot = createRobotForLocalizationBasic();

    robot.initialize();
    robot.start();
    robot.setPose(Pose2D{0.0, 0.0, 0.0});

    // This is a localization-only test. Do not command autonomous motion here.
    //
    // Physical checks:
    // 1. Rotate robot counterclockwise by hand.
    //    Expected: theta increases.
    //
    // 2. Push robot forward by hand while theta is near 0.
    //    Expected: x increases.
    //
    // 3. Push robot backward by hand.
    //    Expected: x decreases.
    //
    // 4. If a lateral tracker is installed, slide the robot sideways by hand.
    //    Expected: y changes with the configured lateral sign.
    //
    // Common failures:
    // - Heading sign wrong: check IMU orientation/yaw convention.
    // - Forward gives negative x: reverse vertical tracker.
    // - x/y swapped: check tracker axis configuration.

    while (true) {
        const Pose2D pose = robot.getPose();
        printPose(pose);

        pros::lcd::print(4, "Move robot by hand.");
        pros::lcd::print(5, "No motor motion here.");

        pros::delay(50);
    }
}