#pragma once

#include <cstdint>
#include <optional>

#include "pros/distance.hpp"

namespace studentlib {

class DistanceSensor {
public:
    explicit DistanceSensor(std::uint8_t port);

    std::optional<double> getDistanceInches() const;

private:
    mutable pros::Distance sensor_;
};

}  // namespace studentlib