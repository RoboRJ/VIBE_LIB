#include "studentlib/api/builder.hpp"
#include "studentlib/math/pose.hpp"

using namespace studentlib;

// Smallest plausible differential/tank configuration.
//
// This is useful for manual driving, early bring-up, and testing motor directions.
// It does not add tracker wheels, distance sensors, or a field map.
// With this setup, localization is very limited compared with odometry/MCL examples.
void configMinimalTankRobotExample() {
    auto robot = Builder()
        .withTankDrive(
            {1, 2},     // left motor ports
            {3, 4},     // right motor ports
            12.0,       // track width in inches
            1.625)      // drive wheel radius in inches
        .build();

    robot.initialize();
    robot.setPose(Pose2D{0.0, 0.0, 0.0});

    // Example manual command
    robot.tank(0.3, 0.3);
}