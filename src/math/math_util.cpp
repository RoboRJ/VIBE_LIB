#include "studentlib/math/math_util.hpp"

#include <cmath>

namespace studentlib {

double clamp(double value, double min_value, double max_value) {
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

double sign(double value) {
    if (value > 0.0) {
        return 1.0;
    }

    if (value < 0.0) {
        return -1.0;
    }

    return 0.0;
}

double square(double value) {
    return value * value;
}

double lerp(double start, double end, double t) {
    return start + (end - start) * t;
}

bool nearlyEqual(double a, double b, double epsilon) {
    return std::abs(a - b) <= epsilon;
}

double dot(const Vector2D& a, const Vector2D& b) {
    return a.x * b.x + a.y * b.y;
}

double cross(const Vector2D& a, const Vector2D& b) {
    return a.x * b.y - a.y * b.x;
}

double magnitudeSquared(const Vector2D& value) {
    return dot(value, value);
}

double magnitude(const Vector2D& value) {
    return std::sqrt(magnitudeSquared(value));
}

Vector2D normalized(const Vector2D& value) {
    const double length = magnitude(value);

    if (length <= 1e-9) {
        return Vector2D{0.0, 0.0};
    }

    return value / length;
}

double distanceSquared(const Point2D& a, const Point2D& b) {
    return square(a.x - b.x) + square(a.y - b.y);
}

double distance(const Point2D& a, const Point2D& b) {
    return std::sqrt(distanceSquared(a, b));
}

double headingToPoint(const Point2D& from, const Point2D& to) {
    return std::atan2(to.y - from.y, to.x - from.x);
}

Vector2D headingVector(double heading_radians) {
    return Vector2D{std::cos(heading_radians), std::sin(heading_radians)};
}

Vector2D rotateVector(const Vector2D& value, double angle_radians) {
    const double c = std::cos(angle_radians);
    const double s = std::sin(angle_radians);

    return Vector2D{
        value.x * c - value.y * s,
        value.x * s + value.y * c
    };
}

Point2D rotatePoint(const Point2D& value, double angle_radians) {
    const double c = std::cos(angle_radians);
    const double s = std::sin(angle_radians);

    return Point2D{
        value.x * c - value.y * s,
        value.x * s + value.y * c
    };
}

Point2D transformPoint(const Pose2D& pose, const Point2D& local_point) {
    const Point2D rotated = rotatePoint(local_point, pose.theta);

    return Point2D{
        pose.x + rotated.x,
        pose.y + rotated.y
    };
}

Point2D inverseTransformPoint(const Pose2D& pose, const Point2D& global_point) {
    const Point2D shifted{
        global_point.x - pose.x,
        global_point.y - pose.y
    };

    return rotatePoint(shifted, -pose.theta);
}

}  // namespace studentlib