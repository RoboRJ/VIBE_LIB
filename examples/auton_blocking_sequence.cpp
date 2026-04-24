#include "studentlib/api/builder.hpp"
#include "studentlib/math/point.hpp"
#include "studentlib/math/pose.hpp"

using namespace studentlib;

void autonomous() {
    auto robot = Builder()
        .withTankDrive({1, 2}, {3, 4}, 12.0, 1.625)
        .withIMU(5)
        .withVerticalTracker(6, 1.0, 1.0, Point2D{0.0, 0.0}, false)
        .withHorizontalTracker(7, 1.0, 1.0, Point2D{0.0, 0.0}, false)
        .build();

    robot.initialize();
    robot.setPose(Pose2D{0.0, 0.0, 0.0});

    // Each blocking call waits for completion before the next line runs.
    robot.moveToPose(Pose2D{24.0, 0.0, 0.0});
    robot.moveToPose(Pose2D{36.0, 18.0, 1.57});
    robot.turnToPoint(Point2D{60.0, 24.0});
    robot.driveToPoint(Point2D{60.0, 36.0});

    // If one of these motions times out, the call still returns.
    // Checking status afterward is a simple way to catch that.
    MotionStatus final_status = robot.getMotionStatus();
    (void)final_status;
}