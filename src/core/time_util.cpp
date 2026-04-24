#include "studentlib/core/time_util.hpp"

#include "pros/rtos.hpp"
#include "studentlib/core/units.hpp"

namespace studentlib {

timestamp_ms_t nowMilliseconds() {
    return pros::millis();
}

double nowSeconds() {
    return millisecondsToSeconds(static_cast<double>(nowMilliseconds()));
}

void delayMilliseconds(timestamp_ms_t delay_ms) {
    pros::delay(delay_ms);
}

void delaySeconds(double delay_seconds) {
    if (delay_seconds <= 0.0) {
        return;
    }

    const auto delay_ms =
        static_cast<timestamp_ms_t>(secondsToMilliseconds(delay_seconds));

    delayMilliseconds(delay_ms);
}

bool hasElapsed(timestamp_ms_t start_time_ms, timestamp_ms_t duration_ms) {
    return static_cast<timestamp_ms_t>(nowMilliseconds() - start_time_ms) >=
           duration_ms;
}

}  // namespace studentlib