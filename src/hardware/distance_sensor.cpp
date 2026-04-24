#include "studentlib/hardware/distance_sensor.hpp"

namespace studentlib {

namespace {
constexpr double kMillimetersPerInch = 25.4;
}  // namespace

DistanceSensor::DistanceSensor(std::uint8_t port)
    : sensor_(port) {}

std::optional<double> DistanceSensor::getDistanceInches() const {
    const double distance_millimeters = sensor_.get();

    if (distance_millimeters <= 0.0) {
        return std::nullopt;
    }

    return distance_millimeters / kMillimetersPerInch;
}

}  // namespace studentlib