#include "studentlib/motion/quadratic_bezier_path.hpp"

#include <cmath>

#include "studentlib/math/math_util.hpp"

namespace studentlib {

QuadraticBezierPath::QuadraticBezierPath(
    const Pose2D& startPose,
    const Pose2D& targetPose,
    const QuadraticBezierPathParams& params)
    : start_pose_(startPose),
      target_pose_(targetPose),
      params_(params) {
    const Point2D start_point = start_pose_.position();
    const Point2D target_point = target_pose_.position();

    const double start_to_target_distance = distance(start_point, target_point);
    const double lead_distance =
        std::max(0.0, params_.leadFactor) * start_to_target_distance;

    // The project heading convention is:
    // x points along cos(theta), y points along sin(theta).
    //
    // To place the control point "behind" the final pose, move backward from the
    // target point along the negative final-heading direction.
    const Vector2D heading_offset = headingVector(target_pose_.theta) * lead_distance;

    control_point_ = Point2D{
        target_point.x - heading_offset.x,
        target_point.y - heading_offset.y
    };
}

const Pose2D& QuadraticBezierPath::getStartPose() const {
    return start_pose_;
}

const Pose2D& QuadraticBezierPath::getTargetPose() const {
    return target_pose_;
}

Point2D QuadraticBezierPath::getControlPoint() const {
    return control_point_;
}

Point2D QuadraticBezierPath::samplePoint(double t) const {
    const double clamped_t = clampT(t);
    const double one_minus_t = 1.0 - clamped_t;

    const Point2D p0 = start_pose_.position();
    const Point2D p1 = control_point_;
    const Point2D p2 = target_pose_.position();

    return Point2D{
        one_minus_t * one_minus_t * p0.x +
            2.0 * one_minus_t * clamped_t * p1.x +
            clamped_t * clamped_t * p2.x,
        one_minus_t * one_minus_t * p0.y +
            2.0 * one_minus_t * clamped_t * p1.y +
            clamped_t * clamped_t * p2.y
    };
}

Vector2D QuadraticBezierPath::sampleTangent(double t) const {
    const double clamped_t = clampT(t);
    const double one_minus_t = 1.0 - clamped_t;

    const Point2D p0 = start_pose_.position();
    const Point2D p1 = control_point_;
    const Point2D p2 = target_pose_.position();

    return Vector2D{
        2.0 * one_minus_t * (p1.x - p0.x) +
            2.0 * clamped_t * (p2.x - p1.x),
        2.0 * one_minus_t * (p1.y - p0.y) +
            2.0 * clamped_t * (p2.y - p1.y)
    };
}

double QuadraticBezierPath::sampleHeading(double t) const {
    const Vector2D tangent = sampleTangent(t);

    if (magnitudeSquared(tangent) <= 1e-12) {
        return target_pose_.theta;
    }

    return std::atan2(tangent.y, tangent.x);
}

double QuadraticBezierPath::findNearestT(
    const Point2D& position,
    double tMin,
    double tMax,
    std::size_t samples) const {

    double search_min = clampT(tMin);
    double search_max = clampT(tMax);

    if (search_max < search_min) {
        const double temp = search_min;
        search_min = search_max;
        search_max = temp;
    }

    const std::size_t sample_count = (samples < 2) ? 2 : samples;

    double best_t = search_min;
    double best_distance_squared = distanceSquared(position, samplePoint(search_min));

    for (std::size_t i = 1; i < sample_count; ++i) {
        const double fraction =
            static_cast<double>(i) / static_cast<double>(sample_count - 1);

        const double t =
            search_min + fraction * (search_max - search_min);

        const double candidate_distance_squared =
            distanceSquared(position, samplePoint(t));

        if (candidate_distance_squared < best_distance_squared) {
            best_distance_squared = candidate_distance_squared;
            best_t = t;
        }
    }

    return best_t;
}

double QuadraticBezierPath::clampT(double t) {
    if (t < 0.0) {
        return 0.0;
    }

    if (t > 1.0) {
        return 1.0;
    }

    return t;
}

}  // namespace studentlib