#pragma once

#include <optional>
#include <vector>

#include "studentlib/math/point.hpp"

namespace studentlib {

struct LineSegment2D {
    Point2D start{};
    Point2D end{};
};

struct FieldBounds {
    double minX{0.0};
    double maxX{0.0};
    double minY{0.0};
    double maxY{0.0};
};

class FieldMap {
public:
    FieldMap() = default;

    void addSegment(const LineSegment2D& segment);
    const std::vector<LineSegment2D>& getSegments() const;

    static FieldMap makeRectangularField(double widthInches, double heightInches);

    std::optional<double> raycastDistance(
        const Point2D& origin,
        double directionRadians,
        double maxDistanceInches) const;

    std::optional<FieldBounds> getBounds() const;

private:
    std::vector<LineSegment2D> segments_{};
};

}  // namespace studentlib