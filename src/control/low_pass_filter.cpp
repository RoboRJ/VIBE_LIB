#include "studentlib/control/low_pass_filter.hpp"

namespace studentlib {

LowPassFilter::LowPassFilter(double alpha)
    : alpha_(clampAlpha(alpha)) {}

double LowPassFilter::update(double input) {
    if (!has_value_) {
        value_ = input;
        has_value_ = true;
        return value_;
    }

    value_ = alpha_ * input + (1.0 - alpha_) * value_;
    return value_;
}

void LowPassFilter::reset() {
    value_ = 0.0;
    has_value_ = false;
}

void LowPassFilter::reset(double value) {
    value_ = value;
    has_value_ = true;
}

double LowPassFilter::getValue() const {
    return value_;
}

bool LowPassFilter::hasValue() const {
    return has_value_;
}

void LowPassFilter::setAlpha(double alpha) {
    alpha_ = clampAlpha(alpha);
}

double LowPassFilter::getAlpha() const {
    return alpha_;
}

double LowPassFilter::clampAlpha(double alpha) {
    if (alpha < 0.0) {
        return 0.0;
    }

    if (alpha > 1.0) {
        return 1.0;
    }

    return alpha;
}

}  // namespace studentlib