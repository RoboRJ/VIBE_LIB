#include "studentlib/api/builder.hpp"
#include "studentlib/math/point.hpp"
#include "studentlib/math/pose.hpp"
#include "studentlib/robot/robot_config.hpp"

using namespace studentlib;

// Practical odometry-focused configuration.
//
// Added beyond the minimal setup:
// - IMU: gives heading
// - one longitudinal tracker: forward/backward displacement
// - one lateral tracker: sideways displacement
//
// This is the most practical default if you want solid localization
// without the extra map and distance-sensor setup needed for MCL.
void configOdometryRobotExample() {
    auto robot = Builder()
        .withTankDrive(
            {1, 2},
            {3, 4},
            12.0,
            1.625)
        .withIMU(5)  // heading sensor
        .withVerticalTracker(
            6,                 // rotation sensor port
            1.0,               // tracker wheel radius
            1.0,               // gear ratio
            Point2D{0.0, 0.0}, // position in robot frame
            false)
        .withHorizontalTracker(
            7,
            1.0,
            1.0,
            Point2D{0.0, 0.0},
            false)
        .withLocalizationMode(LocalizationMode::ODOMETRY)
        .build();

    robot.initialize();
    robot.setPose(Pose2D{0.0, 0.0, 0.0});

    Pose2D pose = robot.getPose();
    (void)pose;
}