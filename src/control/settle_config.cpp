#include "studentlib/control/settle_config.hpp"

namespace studentlib {

bool SettleConfig::isValid() const {
    const bool velocity_tolerance_valid =
        (velocityTolerance < 0.0) || (velocityTolerance >= 0.0);

    return positionTolerance >= 0.0 &&
           velocity_tolerance_valid &&
           settleTimeSeconds >= 0.0 &&
           timeoutSeconds >= 0.0;
}

}  // namespace studentlib