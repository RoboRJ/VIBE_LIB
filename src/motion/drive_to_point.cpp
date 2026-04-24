#include "studentlib/motion/drive_to_point.hpp"

#include <algorithm>
#include <cmath>

#include "studentlib/math/angle.hpp"
#include "studentlib/math/math_util.hpp"

namespace studentlib {

namespace {

double clampMagnitude(double value, double maxMagnitude) {
    const double limit = std::abs(maxMagnitude);

    if (std::abs(value) > limit) {
        return sign(value) * limit;
    }

    return value;
}

double applyMinimumMagnitude(double value, double minimumMagnitude) {
    const double minimum = std::abs(minimumMagnitude);

    if (value == 0.0) {
        return 0.0;
    }

    if (std::abs(value) < minimum) {
        return sign(value) * minimum;
    }

    return value;
}

void scaleTankCommands(double& left, double& right, double maxMagnitude) {
    const double limit = std::abs(maxMagnitude);
    const double largest = std::max(std::abs(left), std::abs(right));

    if (largest > limit && largest > 0.0) {
        const double scale = limit / largest;
        left *= scale;
        right *= scale;
    }
}

bool isFinishedStatus(MotionStatus status) {
    return status == MotionStatus::SETTLED ||
           status == MotionStatus::TIMED_OUT ||
           status == MotionStatus::CANCELED ||
           status == MotionStatus::ERROR;
}

}  // namespace

DriveToPoint::DriveToPoint(
    Drivetrain& drivetrain,
    StateEstimator& estimator,
    const Point2D& target,
    const DriveToPointParams& params)
    : drivetrain_(drivetrain),
      estimator_(estimator),
      linear_controller_(params.linear),
      angular_controller_(params.angular),
      target_(target),
      params_(params) {}

void DriveToPoint::start() {
    started_ = true;
    status_ = MotionStatus::RUNNING;
    elapsed_time_seconds_ = 0.0;
    time_within_tolerance_seconds_ = 0.0;
    linear_controller_.reset();
    angular_controller_.reset();
    drivetrain_.stop();
}

void DriveToPoint::update(double dt) {
    if (!started_ || status_ != MotionStatus::RUNNING) {
        return;
    }

    if (dt > 0.0) {
        elapsed_time_seconds_ += dt;
    }

    const Pose2D pose = estimator_.getPose();
    const Twist2D velocity = estimator_.getVelocity();

    const double position_tolerance = std::max(0.0, params_.settle.positionTolerance);
    const double settle_time = std::max(0.0, params_.settle.settleTimeSeconds);
    const double timeout = params_.settle.timeoutSeconds;

    const double dx = target_.x - pose.x;
    const double dy = target_.y - pose.y;
    const double distance_error = std::sqrt(dx * dx + dy * dy);

    double desired_heading = pose.theta;

    if (distance_error > 1e-9) {
        desired_heading = std::atan2(dy, dx);
    }

    if (params_.reverse) {
        desired_heading = normalizeAngle(desired_heading + kPi);
    }

    const double heading_error =
        normalizeAngle(desired_heading - pose.theta);

    double linear_command = linear_controller_.updateFromError(distance_error, dt);
    double angular_command = angular_controller_.updateFromError(heading_error, dt);

    if (params_.reverse) {
        linear_command = -linear_command;
    }

    linear_command = clampMagnitude(linear_command, params_.maxSpeed);
    angular_command = clampMagnitude(angular_command, params_.maxSpeed);

    if (distance_error > position_tolerance) {
        linear_command = applyMinimumMagnitude(linear_command, params_.minSpeed);
    } else {
        linear_command = 0.0;
        angular_command = 0.0;
    }

    double left_command = linear_command - angular_command;
    double right_command = linear_command + angular_command;

    scaleTankCommands(left_command, right_command, params_.maxSpeed);
    drivetrain_.tank(left_command, right_command);

    bool within_tolerance = distance_error <= position_tolerance;

    if (params_.settle.velocityTolerance >= 0.0) {
        const double translational_speed =
            std::sqrt(velocity.vx * velocity.vx + velocity.vy * velocity.vy);

        within_tolerance =
            within_tolerance &&
            (translational_speed <= params_.settle.velocityTolerance);
    }

    if (within_tolerance) {
        if (settle_time <= 0.0) {
            status_ = MotionStatus::SETTLED;
            drivetrain_.stop();
            return;
        }

        if (dt > 0.0) {
            time_within_tolerance_seconds_ += dt;
        }
    } else {
        time_within_tolerance_seconds_ = 0.0;
    }

    if (time_within_tolerance_seconds_ >= settle_time) {
        status_ = MotionStatus::SETTLED;
        drivetrain_.stop();
        return;
    }

    if (timeout > 0.0 && elapsed_time_seconds_ >= timeout) {
        status_ = MotionStatus::TIMED_OUT;
        drivetrain_.stop();
    }
}

bool DriveToPoint::isFinished() const {
    return isFinishedStatus(status_);
}

MotionStatus DriveToPoint::getStatus() const {
    return status_;
}

void DriveToPoint::cancel() {
    if (isFinished()) {
        return;
    }

    status_ = MotionStatus::CANCELED;
    drivetrain_.stop();
}

}  // namespace studentlib