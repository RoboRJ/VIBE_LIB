#pragma once

#include <cstddef>
#include <vector>

#include "pros/motors.hpp"

namespace studentlib {

class MotorGroup {
public:
    explicit MotorGroup(const std::vector<int>& motorPorts);
    MotorGroup(const std::vector<int>& motorPorts,
               const std::vector<bool>& reversedFlags);

    void setBrakeMode(pros::motor_brake_mode_e_t mode);
    void setVoltage(double millivolts);
    void setVelocity(double rpm);
    void stop();

    void tarePosition();

    double getAveragePositionRadians() const;
    double getAverageVelocityRadPerSec() const;

    std::size_t size() const;

private:
    std::vector<pros::Motor> motors_;
    std::vector<bool> reversed_flags_;

    void buildMotors(const std::vector<int>& motorPorts,
                     const std::vector<bool>& reversedFlags);

    static double clampMillivolts(double millivolts);
};

}  // namespace studentlib