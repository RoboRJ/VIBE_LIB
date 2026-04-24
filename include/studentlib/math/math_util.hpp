#pragma once

#include "studentlib/math/pose.hpp"

namespace studentlib {

double clamp(double value, double min_value, double max_value);
double sign(double value);
double square(double value);
double lerp(double start, double end, double t);
bool nearlyEqual(double a, double b, double epsilon = 1e-9);

double dot(const Vector2D& a, const Vector2D& b);
double cross(const Vector2D& a, const Vector2D& b);

double magnitudeSquared(const Vector2D& value);
double magnitude(const Vector2D& value);
Vector2D normalized(const Vector2D& value);

double distanceSquared(const Point2D& a, const Point2D& b);
double distance(const Point2D& a, const Point2D& b);

double headingToPoint(const Point2D& from, const Point2D& to);
Vector2D headingVector(double heading_radians);

Vector2D rotateVector(const Vector2D& value, double angle_radians);
Point2D rotatePoint(const Point2D& value, double angle_radians);

Point2D transformPoint(const Pose2D& pose, const Point2D& local_point);
Point2D inverseTransformPoint(const Pose2D& pose, const Point2D& global_point);

}  // namespace studentlib