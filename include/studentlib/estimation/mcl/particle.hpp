#pragma once

#include "studentlib/math/pose.hpp"

namespace studentlib {

// weight is the particle importance weight used during reweighting/resampling.
struct Particle {
    Pose2D pose{};
    double weight{1.0};
};

}  // namespace studentlib