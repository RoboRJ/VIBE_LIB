#pragma once

#include "studentlib/math/point.hpp"

namespace studentlib {

struct Pose2D {
    double x{0.0};
    double y{0.0};
    double theta{0.0};

    Pose2D() = default;

    Pose2D(double x_in, double y_in, double theta_in)
        : x(x_in), y(y_in), theta(theta_in) {}

    Pose2D(const Point2D& position, double theta_in)
        : x(position.x), y(position.y), theta(theta_in) {}

    Point2D position() const {
        return Point2D{x, y};
    }
};

}  // namespace studentlib