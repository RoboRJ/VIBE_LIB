#include "studentlib/estimation/mcl/motion_model.hpp"

#include <cmath>
#include <random>

#include "studentlib/math/angle.hpp"

namespace studentlib {

MotionModel::MotionModel(const MCLConfig& config)
    : config_(config),
      rng_(std::random_device{}()) {}

void MotionModel::apply(Particle& particle, const MotionDelta& delta) const {
    const double noisy_dx_local =
        delta.dxLocalInches + sampleGaussian(config_.translationNoiseStdDevInches);

    const double noisy_dy_local =
        delta.dyLocalInches + sampleGaussian(config_.translationNoiseStdDevInches);

    const double noisy_dtheta =
        delta.dthetaRadians + sampleGaussian(config_.rotationNoiseStdDevRadians);

    const double theta_mid = particle.pose.theta + (noisy_dtheta * 0.5);

    const double dx_world =
        noisy_dx_local * std::cos(theta_mid) -
        noisy_dy_local * std::sin(theta_mid);

    const double dy_world =
        noisy_dx_local * std::sin(theta_mid) +
        noisy_dy_local * std::cos(theta_mid);

    particle.pose.x += dx_world;
    particle.pose.y += dy_world;
    particle.pose.theta = normalizeAngle(particle.pose.theta + noisy_dtheta);
}

double MotionModel::sampleGaussian(double standardDeviation) const {
    if (standardDeviation <= 0.0) {
        return 0.0;
    }

    std::normal_distribution<double> distribution(0.0, standardDeviation);
    return distribution(rng_);
}

}  // namespace studentlib