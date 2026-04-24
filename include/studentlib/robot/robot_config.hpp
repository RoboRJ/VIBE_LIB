#pragma once

namespace studentlib {

enum class LocalizationMode {
    ODOMETRY,
    MCL
};

enum class ChassisModel {
    DIFFERENTIAL,
    HOLONOMIC
};

// v1 fully supports differential/tank-drive behavior.
// HOLONOMIC is included as a future configuration hook.
struct RobotConfig {
    LocalizationMode localizationMode{LocalizationMode::ODOMETRY};
    ChassisModel chassisModel{ChassisModel::DIFFERENTIAL};
    double localizationPeriodSeconds{0.01};
    double motionPeriodSeconds{0.01};

    bool isValid() const;
};

}  // namespace studentlib