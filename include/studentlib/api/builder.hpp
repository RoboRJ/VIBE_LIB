#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "studentlib/math/point.hpp"
#include "studentlib/math/pose.hpp"
#include "studentlib/model/field_map.hpp"
#include "studentlib/robot/robot.hpp"
#include "studentlib/robot/robot_config.hpp"

namespace studentlib {

class Builder {
public:
    Builder() = default;

    Builder& withTankDrive(
        const std::vector<int>& leftMotorPorts,
        const std::vector<int>& rightMotorPorts,
        double trackWidthInches,
        double wheelRadiusInches,
        double gearRatio = 1.0,
        double maxVoltageMillivolts = 12000.0);

    Builder& withIMU(std::int8_t port, double yawOffsetRadians = 0.0);

    Builder& withVerticalTracker(
        std::int8_t port,
        double wheelRadiusInches,
        double gearRatio,
        const Point2D& positionInRobotFrame = {},
        bool reversed = false);

    Builder& withHorizontalTracker(
        std::int8_t port,
        double wheelRadiusInches,
        double gearRatio,
        const Point2D& positionInRobotFrame = {},
        bool reversed = false);

    Builder& withDistanceSensor(
        std::uint8_t port,
        const Pose2D& poseInRobotFrame,
        double maxRangeInches,
        double beamAngleOffsetRadians = 0.0);

    Builder& withFieldMap(const FieldMap& fieldMap);
    Builder& withLocalizationMode(LocalizationMode mode);
    Builder& withLocalizationPeriod(double periodSeconds);
    Builder& withMotionPeriod(double periodSeconds);
    Builder& withChassisModel(ChassisModel model);

    Robot build() const;

private:
    RobotConfig robot_config_{};
    DrivetrainConfig drivetrain_config_{};

    std::vector<int> left_motor_ports_{};
    std::vector<int> right_motor_ports_{};

    std::optional<RobotIMUConfig> imu_config_{};
    std::vector<RobotTrackerConfig> tracker_configs_{};
    std::vector<RobotDistanceSensorConfig> distance_sensor_configs_{};
    std::optional<FieldMap> field_map_{};
};

}  // namespace studentlib