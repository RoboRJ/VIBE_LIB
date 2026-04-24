#include "studentlib/estimation/mcl/resampler.hpp"

#include <cmath>
#include <random>
#include <vector>

#include "studentlib/math/angle.hpp"

namespace studentlib {

Resampler::Resampler(const MCLConfig& config)
    : config_(config),
      rng_(std::random_device{}()) {}

void Resampler::normalizeWeights(std::vector<Particle>& particles) const {
    if (particles.empty()) {
        return;
    }

    double weight_sum = 0.0;

    for (const auto& particle : particles) {
        if (std::isfinite(particle.weight) && particle.weight > 0.0) {
            weight_sum += particle.weight;
        }
    }

    if (!std::isfinite(weight_sum) || weight_sum <= 0.0) {
        const double uniform_weight = 1.0 / static_cast<double>(particles.size());

        for (auto& particle : particles) {
            particle.weight = uniform_weight;
        }

        return;
    }

    for (auto& particle : particles) {
        if (!std::isfinite(particle.weight) || particle.weight < 0.0) {
            particle.weight = 0.0;
        }

        particle.weight /= weight_sum;
    }
}

double Resampler::computeEffectiveParticleCount(const std::vector<Particle>& particles) const {
    if (particles.empty()) {
        return 0.0;
    }

    double squared_weight_sum = 0.0;

    for (const auto& particle : particles) {
        squared_weight_sum += particle.weight * particle.weight;
    }

    if (squared_weight_sum <= 0.0) {
        return 0.0;
    }

    return 1.0 / squared_weight_sum;
}

void Resampler::resample(std::vector<Particle>& particles, const FieldMap& fieldMap) const {
    if (particles.empty()) {
        return;
    }

    normalizeWeights(particles);

    const std::size_t particle_count = particles.size();
    const double step = 1.0 / static_cast<double>(particle_count);

    std::uniform_real_distribution<double> offset_distribution(0.0, step);
    const double initial_offset = offset_distribution(rng_);

    std::vector<Particle> new_particles;
    new_particles.reserve(particle_count);

    std::size_t source_index = 0;
    double cumulative_weight = particles[0].weight;

    for (std::size_t m = 0; m < particle_count; ++m) {
        const double target = initial_offset + static_cast<double>(m) * step;

        while ((source_index + 1) < particle_count && target > cumulative_weight) {
            ++source_index;
            cumulative_weight += particles[source_index].weight;
        }

        new_particles.push_back(particles[source_index]);
    }

    const std::optional<FieldBounds> bounds = fieldMap.getBounds();

    if (bounds.has_value() && config_.randomParticleInjectionRatio > 0.0) {
        const std::size_t injection_count = static_cast<std::size_t>(
            config_.randomParticleInjectionRatio * static_cast<double>(particle_count));

        for (std::size_t i = 0; i < injection_count && i < new_particles.size(); ++i) {
            const std::size_t replace_index = new_particles.size() - 1 - i;
            new_particles[replace_index] = makeRandomParticle(*bounds);
        }
    }

    const double uniform_weight = 1.0 / static_cast<double>(particle_count);

    for (auto& particle : new_particles) {
        particle.weight = uniform_weight;
    }

    particles = std::move(new_particles);
}

Particle Resampler::makeRandomParticle(const FieldBounds& bounds) const {
    std::uniform_real_distribution<double> x_distribution(bounds.minX, bounds.maxX);
    std::uniform_real_distribution<double> y_distribution(bounds.minY, bounds.maxY);
    std::uniform_real_distribution<double> theta_distribution(-kPi, kPi);

    Particle particle;
    particle.pose.x = x_distribution(rng_);
    particle.pose.y = y_distribution(rng_);
    particle.pose.theta = theta_distribution(rng_);
    particle.weight = 1.0;

    return particle;
}

}  // namespace studentlib