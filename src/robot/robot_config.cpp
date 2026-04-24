#include "studentlib/robot/robot_config.hpp"

namespace studentlib {

bool RobotConfig::isValid() const {
    return localizationPeriodSeconds > 0.0 &&
           motionPeriodSeconds > 0.0;
}

}  // namespace studentlib