#pragma once

#include "studentlib/motion/motion_status.hpp"

namespace studentlib {

class MotionCommand {
public:
    virtual void start() = 0;
    virtual void update(double dt) = 0;
    virtual bool isFinished() const = 0;
    virtual MotionStatus getStatus() const = 0;
    virtual void cancel() = 0;
    virtual ~MotionCommand() = default;
};

}  // namespace studentlib