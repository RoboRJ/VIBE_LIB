#pragma once

namespace studentlib {

// tank(...) and arcade(...) use normalized command-style inputs.
// In v1, the expected convention is typically in the range [-1, 1].
//
// setVoltage(...) is the explicit low-level millivolt command interface.

class Drivetrain {
public:
    virtual void tank(double left, double right) = 0;
    virtual void arcade(double forward, double turn) = 0;
    virtual void setVoltage(double leftMillivolts, double rightMillivolts) = 0;
    virtual void stop() = 0;
    virtual ~Drivetrain() = default;
};

}  // namespace studentlib