#pragma once

namespace studentlib {

using Distance = double;
using Radians = double;
using Seconds = double;
using Millivolts = double;

constexpr double kInchesPerFoot = 12.0;
constexpr double kInchesPerMeter = 39.37007874015748;
constexpr double kMetersPerInch = 1.0 / kInchesPerMeter;

constexpr double kMillisecondsPerSecond = 1000.0;
constexpr double kSecondsPerMillisecond = 1.0 / kMillisecondsPerSecond;

constexpr double kMillivoltsPerVolt = 1000.0;

constexpr double feetToInches(double feet) {
    return feet * kInchesPerFoot;
}

constexpr double inchesToFeet(double inches) {
    return inches / kInchesPerFoot;
}

constexpr double metersToInches(double meters) {
    return meters * kInchesPerMeter;
}

constexpr double inchesToMeters(double inches) {
    return inches * kMetersPerInch;
}

constexpr double voltsToMillivolts(double volts) {
    return volts * kMillivoltsPerVolt;
}

constexpr double millivoltsToVolts(double millivolts) {
    return millivolts / kMillivoltsPerVolt;
}

constexpr double secondsToMilliseconds(double seconds) {
    return seconds * kMillisecondsPerSecond;
}

constexpr double millisecondsToSeconds(double milliseconds) {
    return milliseconds * kSecondsPerMillisecond;
}

}  // namespace studentlib