#pragma once

namespace studentlib {

struct Twist2D {
    double vx{0.0};
    double vy{0.0};
    double omega{0.0};

    Twist2D() = default;

    Twist2D(double vx_in, double vy_in, double omega_in)
        : vx(vx_in), vy(vy_in), omega(omega_in) {}
};

}  // namespace studentlib