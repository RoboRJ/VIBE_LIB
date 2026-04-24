#pragma once

namespace studentlib {

class SlewRateLimiter {
public:
    explicit SlewRateLimiter(double maxRatePerSecond = 0.0);

    double update(double input, double dt);

    void reset();
    void reset(double value);

    double getValue() const;
    bool hasValue() const;

    void setMaxRate(double maxRatePerSecond);
    double getMaxRate() const;

private:
    double max_rate_per_second_{0.0};
    double value_{0.0};
    bool has_value_{false};
};

}  // namespace studentlib