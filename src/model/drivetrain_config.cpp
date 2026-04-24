#include "studentlib/model/drivetrain_config.hpp"

namespace studentlib {

bool DrivetrainConfig::isValid() const {
    return trackWidthInches > 0.0 &&
           wheelRadiusInches > 0.0 &&
           gearRatio > 0.0 &&
           maxVoltageMillivolts > 0.0;
}

}  // namespace studentlib