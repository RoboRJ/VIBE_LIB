#pragma once

#include <random>
#include <vector>

#include "studentlib/estimation/mcl/mcl_config.hpp"
#include "studentlib/estimation/mcl/particle.hpp"
#include "studentlib/model/field_map.hpp"

namespace studentlib {

class Resampler {
public:
    explicit Resampler(const MCLConfig& config);

    void normalizeWeights(std::vector<Particle>& particles) const;
    double computeEffectiveParticleCount(const std::vector<Particle>& particles) const;
    void resample(std::vector<Particle>& particles, const FieldMap& fieldMap) const;

private:
    MCLConfig config_;
    mutable std::mt19937 rng_;

    Particle makeRandomParticle(const FieldBounds& bounds) const;
};

}  // namespace studentlib