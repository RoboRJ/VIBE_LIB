#pragma once

namespace studentlib {

enum class MotionStatus {
    IDLE,
    RUNNING,
    SETTLED,
    TIMED_OUT,
    CANCELED,
    ERROR
};

}  // namespace studentlib