#include "studentlib/api/builder.hpp"
#include "studentlib/math/point.hpp"
#include "studentlib/math/pose.hpp"
#include "studentlib/model/field_map.hpp"
#include "studentlib/robot/robot_config.hpp"

using namespace studentlib;

// Practical MCL-focused configuration.
//
// Added beyond the odometry setup:
// - distance sensors for map-based measurement updates
// - a field map so MCL can predict expected sensor readings
//
// Current Robot behavior: if MCL is requested without a field map,
// the implementation falls back to odometry.
void configMCLRobotExample() {
    FieldMap field = FieldMap::makeRectangularField(144.0, 144.0);

    // Optional internal obstacle/wall segments can be added if useful.
    field.addSegment(LineSegment2D{
        Point2D{72.0, 24.0},
        Point2D{72.0, 60.0}
    });

    auto robot = Builder()
        .withTankDrive(
            {1, 2},
            {3, 4},
            12.0,
            1.625)
        .withIMU(5)
        .withVerticalTracker(6, 1.0, 1.0, Point2D{0.0, 0.0}, false)
        .withHorizontalTracker(7, 1.0, 1.0, Point2D{0.0, 0.0}, false)
        .withDistanceSensor(
            8,
            Pose2D{3.0, 0.0, 0.0},   // front-facing sensor mount
            60.0,
            0.0)
        .withDistanceSensor(
            9,
            Pose2D{0.0, 3.0, 1.5708}, // left-facing sensor mount
            60.0,
            0.0)
        .withFieldMap(field)
        .withLocalizationMode(LocalizationMode::MCL)
        .build();

    robot.initialize();
    robot.setPose(Pose2D{12.0, 12.0, 0.0});

    Pose2D pose = robot.getPose();
    (void)pose;
}