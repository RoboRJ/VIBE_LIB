#include "studentlib/hardware/motor_group.hpp"

#include <cmath>
#include <cstdint>

#include "studentlib/math/angle.hpp"

namespace studentlib {

namespace {
constexpr double kMaxProsVoltageMillivolts = 12000.0;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kRpmToRadPerSec = kTwoPi / 60.0;
}  // namespace

MotorGroup::MotorGroup(const std::vector<int>& motorPorts) {
    buildMotors(motorPorts, {});
}

MotorGroup::MotorGroup(const std::vector<int>& motorPorts,
                       const std::vector<bool>& reversedFlags) {
    buildMotors(motorPorts, reversedFlags);
}

void MotorGroup::setBrakeMode(pros::motor_brake_mode_e_t mode) {
    for (auto& motor : motors_) {
        motor.set_brake_mode(mode);
    }
}

void MotorGroup::setVoltage(double millivolts) {
    const std::int32_t clamped =
        static_cast<std::int32_t>(clampMillivolts(millivolts));

    for (auto& motor : motors_) {
        motor.move_voltage(clamped);
    }
}

void MotorGroup::setVelocity(double rpm) {
    const std::int32_t velocity_command = static_cast<std::int32_t>(rpm);

    for (auto& motor : motors_) {
        motor.move_velocity(velocity_command);
    }
}

void MotorGroup::stop() {
    for (auto& motor : motors_) {
        motor.brake();
    }
}

void MotorGroup::tarePosition() {
    for (auto& motor : motors_) {
        motor.tare_position();
    }
}

double MotorGroup::getAveragePositionRadians() const {
    if (motors_.empty()) {
        return 0.0;
    }

    double sum_radians = 0.0;

    for (const auto& motor : motors_) {
        const double position_degrees = motor.get_position();
        sum_radians += position_degrees * kDegreesToRadians;
    }

    return sum_radians / static_cast<double>(motors_.size());
}

double MotorGroup::getAverageVelocityRadPerSec() const {
    if (motors_.empty()) {
        return 0.0;
    }

    double sum_rad_per_sec = 0.0;

    for (const auto& motor : motors_) {
        const double velocity_rpm = motor.get_actual_velocity();
        sum_rad_per_sec += velocity_rpm * kRpmToRadPerSec;
    }

    return sum_rad_per_sec / static_cast<double>(motors_.size());
}

std::size_t MotorGroup::size() const {
    return motors_.size();
}

void MotorGroup::buildMotors(const std::vector<int>& motorPorts,
                             const std::vector<bool>& reversedFlags) {
    motors_.clear();
    reversed_flags_.clear();

    motors_.reserve(motorPorts.size());
    reversed_flags_.reserve(motorPorts.size());

    for (std::size_t i = 0; i < motorPorts.size(); ++i) {
        const int raw_port = motorPorts[i];
        const bool reversed_from_port = raw_port < 0;
        const bool reversed_from_flags =
            (i < reversedFlags.size()) ? reversedFlags[i] : false;
        const bool reversed = reversed_from_port ^ reversed_from_flags;

        const std::int8_t port =
            static_cast<std::int8_t>(std::abs(raw_port));

        motors_.emplace_back(port);
        motors_.back().set_reversed(reversed);
        reversed_flags_.push_back(reversed);
    }
}

double MotorGroup::clampMillivolts(double millivolts) {
    if (millivolts > kMaxProsVoltageMillivolts) {
        return kMaxProsVoltageMillivolts;
    }

    if (millivolts < -kMaxProsVoltageMillivolts) {
        return -kMaxProsVoltageMillivolts;
    }

    return millivolts;
}

}  // namespace studentlib