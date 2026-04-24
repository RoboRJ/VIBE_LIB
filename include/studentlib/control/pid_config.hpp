#pragma once

namespace studentlib {

// integralClamp limits the magnitude of the accumulated integral state.
// derivativeOnMeasurement switches the derivative term from d(error)/dt
// to -d(measurement)/dt.
// smallErrorThreshold defines the absolute error band used for settled detection.
// smallErrorTimeoutSeconds is how long the error must stay inside that band
// before the controller reports settled.
struct PIDConfig {
    double kP{0.0};
    double kI{0.0};
    double kD{0.0};
    double integralClamp{0.0};
    double outputMin{-1.0};
    double outputMax{1.0};
    double deadband{0.0};
    bool antiWindupEnabled{true};
    bool derivativeOnMeasurement{false};
    double smallErrorThreshold{0.0};
    double smallErrorTimeoutSeconds{0.0};

    bool isValid() const;
};

}  // namespace studentlib