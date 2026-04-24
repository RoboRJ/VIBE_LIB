#pragma once

#include <random>

#include "studentlib/estimation/mcl/mcl_config.hpp"
#include "studentlib/estimation/mcl/particle.hpp"
#include "studentlib/estimation/motion_delta.hpp"

namespace studentlib {

class MotionModel {
public:
    explicit MotionModel(const MCLConfig& config);

    void apply(Particle& particle, const MotionDelta& delta) const;

private:
    MCLConfig config_;
    mutable std::mt19937 rng_;

    double sampleGaussian(double standardDeviation) const;
};

}  // namespace studentlib