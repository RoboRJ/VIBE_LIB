#pragma once

namespace studentlib {

struct Vector2D {
    double x{0.0};
    double y{0.0};

    Vector2D() = default;

    Vector2D(double x_in, double y_in)
        : x(x_in), y(y_in) {}

    Vector2D& operator+=(const Vector2D& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vector2D& operator-=(const Vector2D& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vector2D& operator*=(double scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    Vector2D& operator/=(double scalar) {
        x /= scalar;
        y /= scalar;
        return *this;
    }
};

inline Vector2D operator+(Vector2D left, const Vector2D& right) {
    left += right;
    return left;
}

inline Vector2D operator-(Vector2D left, const Vector2D& right) {
    left -= right;
    return left;
}

inline Vector2D operator-(const Vector2D& value) {
    return Vector2D{-value.x, -value.y};
}

inline Vector2D operator*(Vector2D value, double scalar) {
    value *= scalar;
    return value;
}

inline Vector2D operator*(double scalar, Vector2D value) {
    value *= scalar;
    return value;
}

inline Vector2D operator/(Vector2D value, double scalar) {
    value /= scalar;
    return value;
}

}  // namespace studentlib