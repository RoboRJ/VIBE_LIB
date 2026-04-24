#include "studentlib/control/pid_controller.hpp"

#include <cmath>

namespace studentlib {

PIDController::PIDController(const PIDConfig& config)
    : config_(config) {}

void PIDController::setConfig(const PIDConfig& config) {
    config_ = config;
}

const PIDConfig& PIDController::getConfig() const {
    return config_;
}

void PIDController::reset() {
    integral_accumulator_ = 0.0;
    last_output_ = 0.0;
    last_error_ = 0.0;
    last_measurement_ = 0.0;
    settled_time_seconds_ = 0.0;
    settled_ = false;
    has_last_error_ = false;
    has_last_measurement_ = false;
}

void PIDController::reset(double initialMeasurement) {
    reset();
    last_measurement_ = initialMeasurement;
    has_last_measurement_ = true;
}

double PIDController::update(double setpoint, double measurement, double dt) {
    const double error = setpoint - measurement;

    return updateInternal(
        error,
        true,
        measurement,
        DerivativeMode::AUTO,
        0.0,
        dt);
}

double PIDController::update(double setpoint,
                             double measurement,
                             double measurementRate,
                             double dt) {
    const double error = setpoint - measurement;

    return updateInternal(
        error,
        true,
        measurement,
        DerivativeMode::PROVIDED_MEASUREMENT_RATE,
        measurementRate,
        dt);
}

double PIDController::updateFromError(double error, double dt) {
    return updateInternal(
        error,
        false,
        0.0,
        DerivativeMode::AUTO,
        0.0,
        dt);
}

double PIDController::updateFromError(double error, double errorRate, double dt) {
    return updateInternal(
        error,
        false,
        0.0,
        DerivativeMode::PROVIDED_ERROR_RATE,
        errorRate,
        dt);
}

double PIDController::getLastOutput() const {
    return last_output_;
}

double PIDController::getIntegralTerm() const {
    return integral_accumulator_;
}

double PIDController::getLastError() const {
    return last_error_;
}

bool PIDController::isSettled() const {
    return settled_;
}

bool PIDController::hasState() const {
    return has_last_error_ || has_last_measurement_;
}

double PIDController::updateInternal(double error,
                                     bool hasMeasurement,
                                     double measurement,
                                     DerivativeMode derivativeMode,
                                     double providedDerivative,
                                     double dt) {
    error = applyDeadband(error);

    double p_term = config_.kP * error;
    double d_term = 0.0;

    if (dt > 0.0) {
        if (derivativeMode == DerivativeMode::PROVIDED_ERROR_RATE) {
            d_term = config_.kD * providedDerivative;
        } else if (derivativeMode == DerivativeMode::PROVIDED_MEASUREMENT_RATE) {
            if (config_.derivativeOnMeasurement) {
                d_term = -config_.kD * providedDerivative;
            } else {
                // With only a measurement derivative available, assume the setpoint
                // is locally constant over this update. Then:
                // d(error)/dt = d(setpoint - measurement)/dt = -d(measurement)/dt
                const double error_rate = -providedDerivative;
                d_term = config_.kD * error_rate;
            }
        } else {
            if (config_.derivativeOnMeasurement && hasMeasurement) {
                if (has_last_measurement_) {
                    const double measurement_rate =
                        (measurement - last_measurement_) / dt;
                    d_term = -config_.kD * measurement_rate;
                }
            } else {
                // If no measurement is available, fall back to derivative on error
                // even when derivativeOnMeasurement is enabled.
                if (has_last_error_) {
                    const double error_rate =
                        (error - last_error_) / dt;
                    d_term = config_.kD * error_rate;
                }
            }
        }
    }

    double proposed_integral = integral_accumulator_;

    if (dt > 0.0) {
        proposed_integral += error * dt;
    }

    if (config_.integralClamp > 0.0) {
        if (proposed_integral > config_.integralClamp) {
            proposed_integral = config_.integralClamp;
        }

        if (proposed_integral < -config_.integralClamp) {
            proposed_integral = -config_.integralClamp;
        }
    }

    double i_term = config_.kI * proposed_integral;
    double unclamped_output = p_term + i_term + d_term;

    const double output_min =
        (config_.outputMin <= config_.outputMax) ? config_.outputMin
                                                 : config_.outputMax;
    const double output_max =
        (config_.outputMin <= config_.outputMax) ? config_.outputMax
                                                 : config_.outputMin;

    // Simple anti-windup rule:
    // if the controller is already trying to saturate farther in the same
    // direction as the current error, do not accept the new integral growth.
    if (config_.antiWindupEnabled) {
        const bool saturating_high = unclamped_output > output_max && error > 0.0;
        const bool saturating_low = unclamped_output < output_min && error < 0.0;

        if (saturating_high || saturating_low) {
            i_term = config_.kI * integral_accumulator_;
            unclamped_output = p_term + i_term + d_term;
        } else {
            integral_accumulator_ = proposed_integral;
        }
    } else {
        integral_accumulator_ = proposed_integral;
    }

    const double output = clampOutput(unclamped_output);

    updateSettledState(error, dt);

    last_output_ = output;
    last_error_ = error;
    has_last_error_ = true;

    if (hasMeasurement) {
        last_measurement_ = measurement;
        has_last_measurement_ = true;
    } else {
        has_last_measurement_ = false;
    }

    return last_output_;
}

double PIDController::applyDeadband(double error) const {
    if (std::abs(error) <= config_.deadband) {
        return 0.0;
    }

    return error;
}

double PIDController::clampOutput(double output) const {
    const double output_min =
        (config_.outputMin <= config_.outputMax) ? config_.outputMin
                                                 : config_.outputMax;
    const double output_max =
        (config_.outputMin <= config_.outputMax) ? config_.outputMax
                                                 : config_.outputMin;

    if (output < output_min) {
        return output_min;
    }

    if (output > output_max) {
        return output_max;
    }

    return output;
}

void PIDController::updateSettledState(double error, double dt) {
    if (!settledDetectionEnabled()) {
        settled_time_seconds_ = 0.0;
        settled_ = false;
        return;
    }

    if (std::abs(error) <= config_.smallErrorThreshold) {
        if (dt > 0.0) {
            settled_time_seconds_ += dt;
        }
    } else {
        settled_time_seconds_ = 0.0;
    }

    settled_ = settled_time_seconds_ >= config_.smallErrorTimeoutSeconds;
}

bool PIDController::settledDetectionEnabled() const {
    return config_.smallErrorThreshold > 0.0 &&
           config_.smallErrorTimeoutSeconds > 0.0;
}

}  // namespace studentlib