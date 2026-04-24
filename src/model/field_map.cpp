#include "studentlib/model/field_map.hpp"

#include <cmath>
#include <limits>
#include <optional>

#include "studentlib/math/math_util.hpp"
#include "studentlib/math/vector.hpp"

namespace studentlib {

namespace {
constexpr double kIntersectionEpsilon = 1e-9;
}

void FieldMap::addSegment(const LineSegment2D& segment) {
    segments_.push_back(segment);
}

const std::vector<LineSegment2D>& FieldMap::getSegments() const {
    return segments_;
}

FieldMap FieldMap::makeRectangularField(double widthInches, double heightInches) {
    FieldMap field;

    const Point2D bottom_left{0.0, 0.0};
    const Point2D bottom_right{widthInches, 0.0};
    const Point2D top_right{widthInches, heightInches};
    const Point2D top_left{0.0, heightInches};

    field.addSegment(LineSegment2D{bottom_left, bottom_right});
    field.addSegment(LineSegment2D{bottom_right, top_right});
    field.addSegment(LineSegment2D{top_right, top_left});
    field.addSegment(LineSegment2D{top_left, bottom_left});

    return field;
}

std::optional<double> FieldMap::raycastDistance(
    const Point2D& origin,
    double directionRadians,
    double maxDistanceInches) const {

    if (maxDistanceInches <= 0.0) {
        return std::nullopt;
    }

    const Vector2D ray_direction = headingVector(directionRadians);
    double nearest_distance = std::numeric_limits<double>::infinity();
    bool found_hit = false;

    for (const auto& segment : segments_) {
        const Point2D& p = origin;
        const Point2D& q = segment.start;

        const Vector2D r = ray_direction;
        const Vector2D s = segment.end - segment.start;
        const Vector2D q_minus_p = q - p;

        const double denominator = cross(r, s);

        if (std::abs(denominator) <= kIntersectionEpsilon) {
            continue;
        }

        const double t = cross(q_minus_p, s) / denominator;
        const double u = cross(q_minus_p, r) / denominator;

        const bool hit_on_ray = t >= 0.0;
        const bool hit_on_segment = (u >= 0.0) && (u <= 1.0);
        const bool hit_in_range = t <= maxDistanceInches;

        if (hit_on_ray && hit_on_segment && hit_in_range) {
            if (t < nearest_distance) {
                nearest_distance = t;
                found_hit = true;
            }
        }
    }

    if (!found_hit) {
        return std::nullopt;
    }

    return nearest_distance;
}

std::optional<FieldBounds> FieldMap::getBounds() const {
    if (segments_.empty()) {
        return std::nullopt;
    }

    FieldBounds bounds;
    bool first_point = true;

    for (const auto& segment : segments_) {
        const Point2D points[2] = {segment.start, segment.end};

        for (const Point2D& point : points) {
            if (first_point) {
                bounds.minX = point.x;
                bounds.maxX = point.x;
                bounds.minY = point.y;
                bounds.maxY = point.y;
                first_point = false;
            } else {
                if (point.x < bounds.minX) {
                    bounds.minX = point.x;
                }
                if (point.x > bounds.maxX) {
                    bounds.maxX = point.x;
                }
                if (point.y < bounds.minY) {
                    bounds.minY = point.y;
                }
                if (point.y > bounds.maxY) {
                    bounds.maxY = point.y;
                }
            }
        }
    }

    if (first_point) {
        return std::nullopt;
    }

    return bounds;
}

}  // namespace studentlib