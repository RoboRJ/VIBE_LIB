#pragma once

namespace studentlib {

struct DrivetrainConfig {
    double trackWidthInches{0.0};
    double wheelRadiusInches{0.0};
    double gearRatio{1.0};
    double maxVoltageMillivolts{12000.0};

    bool isValid() const;
};

}  // namespace studentlib