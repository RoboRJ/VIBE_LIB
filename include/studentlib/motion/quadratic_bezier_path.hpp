#pragma once

#include <cstddef>

#include "studentlib/math/point.hpp"
#include "studentlib/math/pose.hpp"
#include "studentlib/math/vector.hpp"
#include "studentlib/motion/motion_params.hpp"

namespace studentlib {

class QuadraticBezierPath {
public:
    QuadraticBezierPath(
        const Pose2D& startPose,
        const Pose2D& targetPose,
        const QuadraticBezierPathParams& params = {});

    const Pose2D& getStartPose() const;
    const Pose2D& getTargetPose() const;
    Point2D getControlPoint() const;

    Point2D samplePoint(double t) const;
    Vector2D sampleTangent(double t) const;
    double sampleHeading(double t) const;

    double findNearestT(
        const Point2D& position,
        double tMin,
        double tMax,
        std::size_t samples = 25) const;

private:
    Pose2D start_pose_{};
    Pose2D target_pose_{};
    Point2D control_point_{};
    QuadraticBezierPathParams params_{};

    static double clampT(double t);
};

}  // namespace studentlib