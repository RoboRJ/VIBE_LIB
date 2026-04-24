#pragma once

#include "studentlib/math/vector.hpp"

namespace studentlib {

struct Point2D {
    double x{0.0};
    double y{0.0};

    Point2D() = default;

    Point2D(double x_in, double y_in)
        : x(x_in), y(y_in) {}
};

inline Point2D operator+(const Point2D& point, const Vector2D& offset) {
    return Point2D{point.x + offset.x, point.y + offset.y};
}

inline Point2D operator+(const Vector2D& offset, const Point2D& point) {
    return point + offset;
}

inline Point2D operator-(const Point2D& point, const Vector2D& offset) {
    return Point2D{point.x - offset.x, point.y - offset.y};
}

inline Vector2D operator-(const Point2D& left, const Point2D& right) {
    return Vector2D{left.x - right.x, left.y - right.y};
}

}  // namespace studentlib