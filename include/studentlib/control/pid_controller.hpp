#pragma once

#include "studentlib/control/pid_config.hpp"

namespace studentlib {

class PIDController {
public:
    explicit PIDController(const PIDConfig& config = {});

    void setConfig(const PIDConfig& config);
    const PIDConfig& getConfig() const;

    void reset();
    void reset(double initialMeasurement);

    double update(double setpoint, double measurement, double dt);
    double update(double setpoint, double measurement, double measurementRate, double dt);

    double updateFromError(double error, double dt);
    double updateFromError(double error, double errorRate, double dt);

    double getLastOutput() const;
    double getIntegralTerm() const;
    double getLastError() const;
    bool isSettled() const;
    bool hasState() const;

private:
    enum class DerivativeMode {
        AUTO,
        PROVIDED_MEASUREMENT_RATE,
        PROVIDED_ERROR_RATE
    };

    PIDConfig config_{};

    double integral_accumulator_{0.0};
    double last_output_{0.0};
    double last_error_{0.0};
    double last_measurement_{0.0};

    double settled_time_seconds_{0.0};
    bool settled_{false};
    bool has_last_error_{false};
    bool has_last_measurement_{false};

    double updateInternal(double error,
                          bool hasMeasurement,
                          double measurement,
                          DerivativeMode derivativeMode,
                          double providedDerivative,
                          double dt);

    double applyDeadband(double error) const;
    double clampOutput(double output) const;
    void updateSettledState(double error, double dt);
    bool settledDetectionEnabled() const;
};

}  // namespace studentlib