#pragma once

namespace studentlib {

class LowPassFilter {
public:
    explicit LowPassFilter(double alpha = 1.0);

    double update(double input);

    void reset();
    void reset(double value);

    double getValue() const;
    bool hasValue() const;

    void setAlpha(double alpha);
    double getAlpha() const;

private:
    double alpha_{1.0};
    double value_{0.0};
    bool has_value_{false};

    static double clampAlpha(double alpha);
};

}  // namespace studentlib