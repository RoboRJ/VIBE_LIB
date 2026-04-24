#include "studentlib/control/pid_config.hpp"

namespace studentlib {

bool PIDConfig::isValid() const {
    return integralClamp >= 0.0 &&
           deadband >= 0.0 &&
           smallErrorThreshold >= 0.0 &&
           smallErrorTimeoutSeconds >= 0.0;
}

}  // namespace studentlib