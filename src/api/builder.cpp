#include "studentlib/api/builder.hpp"

namespace studentlib {

Builder& Builder::withTankDrive(
    const std::vector<int>& leftMotorPorts,
    const std::vector<int>& rightMotorPorts,
    double trackWidthInches,
    double wheelRadiusInches,
    double gearRatio,
    double maxVoltageMillivolts) {

    left_motor_ports_ = leftMotorPorts;
    right_motor_ports_ = rightMotorPorts;

    drivetrain_config_.trackWidthInches = trackWidthInches;
    drivetrain_config_.wheelRadiusInches = wheelRadiusInches;
    drivetrain_config_.gearRatio = gearRatio;
    drivetrain_config_.maxVoltageMillivolts = maxVoltageMillivolts;

    robot_config_.chassisModel = ChassisModel::DIFFERENTIAL;

    return *this;
}

Builder& Builder::withIMU(std::int8_t port, double yawOffsetRadians) {
    imu_config_ = RobotIMUConfig{port, yawOffsetRadians};
    return *this;
}

Builder& Builder::withVerticalTracker(
    std::int8_t port,
    double wheelRadiusInches,
    double gearRatio,
    const Point2D& positionInRobotFrame,
    bool reversed) {

    tracker_configs_.push_back(
        RobotTrackerConfig{
            port,
            reversed,
            wheelRadiusInches,
            gearRatio,
            TrackerAxis::LONGITUDINAL,
            positionInRobotFrame});

    return *this;
}

Builder& Builder::withHorizontalTracker(
    std::int8_t port,
    double wheelRadiusInches,
    double gearRatio,
    const Point2D& positionInRobotFrame,
    bool reversed) {

    tracker_configs_.push_back(
        RobotTrackerConfig{
            port,
            reversed,
            wheelRadiusInches,
            gearRatio,
            TrackerAxis::LATERAL,
            positionInRobotFrame});

    return *this;
}

Builder& Builder::withDistanceSensor(
    std::uint8_t port,
    const Pose2D& poseInRobotFrame,
    double maxRangeInches,
    double beamAngleOffsetRadians) {

    distance_sensor_configs_.push_back(
        RobotDistanceSensorConfig{
            port,
            poseInRobotFrame,
            maxRangeInches,
            beamAngleOffsetRadians});

    return *this;
}

Builder& Builder::withFieldMap(const FieldMap& fieldMap) {
    field_map_ = fieldMap;
    return *this;
}

Builder& Builder::withLocalizationMode(LocalizationMode mode) {
    robot_config_.localizationMode = mode;
    return *this;
}

Builder& Builder::withLocalizationPeriod(double periodSeconds) {
    robot_config_.localizationPeriodSeconds = periodSeconds;
    return *this;
}

Builder& Builder::withMotionPeriod(double periodSeconds) {
    robot_config_.motionPeriodSeconds = periodSeconds;
    return *this;
}

Builder& Builder::withChassisModel(ChassisModel model) {
    robot_config_.chassisModel = model;
    return *this;
}

Robot Builder::build() const {
    return Robot(
        robot_config_,
        drivetrain_config_,
        left_motor_ports_,
        right_motor_ports_,
        imu_config_,
        tracker_configs_,
        distance_sensor_configs_,
        field_map_);
}

}  // namespace studentlib