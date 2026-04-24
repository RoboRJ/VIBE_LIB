#include "studentlib/motion/turn_to_heading.hpp"

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

bool isFinishedStatus(MotionStatus status) {
    return status == MotionStatus::SETTLED ||
           status == MotionStatus::TIMED_OUT ||
           status == MotionStatus::CANCELED ||
           status == MotionStatus::ERROR;
}

}  // namespace

TurnToHeading::TurnToHeading(
    Drivetrain& drivetrain,
    StateEstimator& estimator,
    double targetHeadingRadians,
    const TurnToHeadingParams& params)
    : drivetrain_(drivetrain),
      estimator_(estimator),
      angular_controller_(params.angular),
      target_heading_radians_(normalizeAngle(targetHeadingRadians)),
      params_(params) {}

void TurnToHeading::start() {
    started_ = true;
    status_ = MotionStatus::RUNNING;
    elapsed_time_seconds_ = 0.0;
    time_within_tolerance_seconds_ = 0.0;
    angular_controller_.reset();
    drivetrain_.stop();
}

void TurnToHeading::update(double dt) {
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

    const double heading_error =
        normalizeAngle(target_heading_radians_ - pose.theta);

    double turn_command = angular_controller_.updateFromError(heading_error, dt);
    turn_command = clampMagnitude(turn_command, params_.maxSpeed);

    if (std::abs(heading_error) > position_tolerance) {
        turn_command = applyMinimumMagnitude(turn_command, params_.minSpeed);
    } else {
        turn_command = 0.0;
    }

    drivetrain_.tank(-turn_command, turn_command);

    bool within_tolerance = std::abs(heading_error) <= position_tolerance;

    if (params_.settle.velocityTolerance >= 0.0) {
        within_tolerance =
            within_tolerance &&
            (std::abs(velocity.omega) <= params_.settle.velocityTolerance);
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

bool TurnToHeading::isFinished() const {
    return isFinishedStatus(status_);
}

MotionStatus TurnToHeading::getStatus() const {
    return status_;
}

void TurnToHeading::cancel() {
    if (isFinished()) {
        return;
    }

    status_ = MotionStatus::CANCELED;
    drivetrain_.stop();
}

}  // namespace studentlib