#include "studentlib/control/slew_rate_limiter.hpp"

namespace studentlib {

SlewRateLimiter::SlewRateLimiter(double maxRatePerSecond)
    : max_rate_per_second_(maxRatePerSecond) {}

double SlewRateLimiter::update(double input, double dt) {
    if (!has_value_) {
        value_ = input;
        has_value_ = true;
        return value_;
    }

    if (dt <= 0.0) {
        return value_;
    }

    if (max_rate_per_second_ <= 0.0) {
        value_ = input;
        return value_;
    }

    const double max_delta = max_rate_per_second_ * dt;
    const double desired_delta = input - value_;

    double clamped_delta = desired_delta;

    if (clamped_delta > max_delta) {
        clamped_delta = max_delta;
    }

    if (clamped_delta < -max_delta) {
        clamped_delta = -max_delta;
    }

    value_ += clamped_delta;
    return value_;
}

void SlewRateLimiter::reset() {
    value_ = 0.0;
    has_value_ = false;
}

void SlewRateLimiter::reset(double value) {
    value_ = value;
    has_value_ = true;
}

double SlewRateLimiter::getValue() const {
    return value_;
}

bool SlewRateLimiter::hasValue() const {
    return has_value_;
}

void SlewRateLimiter::setMaxRate(double maxRatePerSecond) {
    max_rate_per_second_ = maxRatePerSecond;
}

double SlewRateLimiter::getMaxRate() const {
    return max_rate_per_second_;
}

}  // namespace studentlib