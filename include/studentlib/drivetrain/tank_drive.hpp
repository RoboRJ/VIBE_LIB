#pragma once

#include "studentlib/drivetrain/drivetrain.hpp"
#include "studentlib/hardware/motor_group.hpp"
#include "studentlib/model/drivetrain_config.hpp"

namespace studentlib {

class TankDrive : public Drivetrain {
public:
    explicit TankDrive(MotorGroup leftMotors,
                       MotorGroup rightMotors,
                       DrivetrainConfig config);

    void tank(double left, double right) override;
    void arcade(double forward, double turn) override;
    void setVoltage(double leftMillivolts, double rightMillivolts) override;
    void stop() override;

    const DrivetrainConfig& getConfig() const;

private:
    MotorGroup left_motors_;
    MotorGroup right_motors_;
    DrivetrainConfig config_;

    static double clampNormalized(double value);
};

}  // namespace studentlib