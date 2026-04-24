#include "studentlib/drivetrain/tank_drive.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace studentlib {

TankDrive::TankDrive(MotorGroup leftMotors,
                     MotorGroup rightMotors,
                     DrivetrainConfig config)
    : left_motors_(std::move(leftMotors)),
      right_motors_(std::move(rightMotors)),
      config_(config) {}

void TankDrive::tank(double left, double right) {
    const double left_command = clampNormalized(left);
    const double right_command = clampNormalized(right);

    const double left_voltage = left_command * config_.maxVoltageMillivolts;
    const double right_voltage = right_command * config_.maxVoltageMillivolts;

    setVoltage(left_voltage, right_voltage);
}

void TankDrive::arcade(double forward, double turn) {
    const double forward_command = clampNormalized(forward);
    const double turn_command = clampNormalized(turn);

    double left_command = forward_command + turn_command;
    double right_command = forward_command - turn_command;

    const double max_magnitude =
        std::max({1.0, std::abs(left_command), std::abs(right_command)});

    left_command /= max_magnitude;
    right_command /= max_magnitude;

    tank(left_command, right_command);
}

void TankDrive::setVoltage(double leftMillivolts, double rightMillivolts) {
    left_motors_.setVoltage(leftMillivolts);
    right_motors_.setVoltage(rightMillivolts);
}

void TankDrive::stop() {
    left_motors_.stop();
    right_motors_.stop();
}

const DrivetrainConfig& TankDrive::getConfig() const {
    return config_;
}

double TankDrive::clampNormalized(double value) {
    if (value > 1.0) {
        return 1.0;
    }

    if (value < -1.0) {
        return -1.0;
    }

    return value;
}

}  // namespace studentlib