#include "studentlib/motion/move_to_pose.hpp"

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

double clampUnit(double value) {
    if (value < 0.0) {
        return 0.0;
    }

    if (value > 1.0) {
        return 1.0;
    }

    return value;
}

double blendAngles(double primaryAngle, double secondaryAngle, double primaryWeight) {
    const double clamped_primary_weight = clampUnit(primaryWeight);
    const double secondary_weight = 1.0 - clamped_primary_weight;

    const double x =
        clamped_primary_weight * std::cos(primaryAngle) +
        secondary_weight * std::cos(secondaryAngle);

    const double y =
        clamped_primary_weight * std::sin(primaryAngle) +
        secondary_weight * std::sin(secondaryAngle);

    if (std::abs(x) < 1e-9 && std::abs(y) < 1e-9) {
        return secondaryAngle;
    }

    return std::atan2(y, x);
}

bool isFinishedStatus(MotionStatus status) {
    return status == MotionStatus::SETTLED ||
           status == MotionStatus::TIMED_OUT ||
           status == MotionStatus::CANCELED ||
           status == MotionStatus::ERROR;
}

double translationalSpeed(const Twist2D& velocity) {
    return std::sqrt(velocity.vx * velocity.vx + velocity.vy * velocity.vy);
}

}  // namespace

MoveToPose::MoveToPose(
    Drivetrain& drivetrain,
    StateEstimator& estimator,
    const Pose2D& target,
    const MoveToPoseParams& params)
    : drivetrain_(drivetrain),
      estimator_(estimator),
      linear_controller_(params.linear),
      angular_controller_(params.angular),
      target_(target),
      params_(params) {
    target_.theta = normalizeAngle(target_.theta);
}

void MoveToPose::start() {
    started_ = true;
    status_ = MotionStatus::RUNNING;
    elapsed_time_seconds_ = 0.0;
    time_within_tolerance_seconds_ = 0.0;
    stage_ = StraightThenTurnStage::DRIVE_TO_POSITION;
    last_t_ = 0.0;

    linear_controller_.reset();
    angular_controller_.reset();

    path_.reset();

    if (params_.pathMode == MoveToPosePathMode::QUADRATIC_BEZIER) {
        const Pose2D start_pose = estimator_.getPose();
        path_.emplace(start_pose, target_, params_.quadraticBezier);
    }

    drivetrain_.stop();
}

void MoveToPose::update(double dt) {
    if (!started_ || status_ != MotionStatus::RUNNING) {
        return;
    }

    if (dt > 0.0) {
        elapsed_time_seconds_ += dt;
    }

    switch (params_.pathMode) {
        case MoveToPosePathMode::POSE_REGULATOR:
            updatePoseRegulator(dt);
            break;

        case MoveToPosePathMode::STRAIGHT_THEN_TURN:
            updateStraightThenTurn(dt);
            break;

        case MoveToPosePathMode::QUADRATIC_BEZIER:
            updateQuadraticBezier(dt);
            break;
    }

    if (status_ != MotionStatus::RUNNING) {
        return;
    }

    if (params_.settle.timeoutSeconds > 0.0 &&
        elapsed_time_seconds_ >= params_.settle.timeoutSeconds) {
        finish(MotionStatus::TIMED_OUT);
    }
}

bool MoveToPose::isFinished() const {
    return isFinishedStatus(status_);
}

MotionStatus MoveToPose::getStatus() const {
    return status_;
}

void MoveToPose::cancel() {
    if (isFinished()) {
        return;
    }

    finish(MotionStatus::CANCELED);
}

void MoveToPose::finish(MotionStatus status) {
    status_ = status;
    drivetrain_.stop();
}

void MoveToPose::commandDriveAndTurn(double linearCommand, double angularCommand) {
    double linear_command = clampMagnitude(linearCommand, params_.maxSpeed);
    double angular_command = clampMagnitude(angularCommand, params_.maxSpeed);

    double left_command = linear_command - angular_command;
    double right_command = linear_command + angular_command;

    scaleTankCommands(left_command, right_command, params_.maxSpeed);
    drivetrain_.tank(left_command, right_command);
}

bool MoveToPose::updateSettleTimer(bool withinTolerance, double dt) {
    if (!withinTolerance) {
        time_within_tolerance_seconds_ = 0.0;
        return false;
    }

    const double settle_time = std::max(0.0, params_.settle.settleTimeSeconds);

    if (settle_time <= 0.0) {
        return true;
    }

    if (dt > 0.0) {
        time_within_tolerance_seconds_ += dt;
    }

    return time_within_tolerance_seconds_ >= settle_time;
}

void MoveToPose::updatePoseRegulator(double dt) {
    const Pose2D pose = estimator_.getPose();
    const Twist2D velocity = estimator_.getVelocity();

    const double position_tolerance = std::max(0.0, params_.settle.positionTolerance);
    const double final_angle_tolerance = std::max(0.0, params_.maxFinalAngleError);

    const double dx = target_.x - pose.x;
    const double dy = target_.y - pose.y;
    const double distance_error = std::sqrt(dx * dx + dy * dy);

    double approach_heading = pose.theta;

    if (distance_error > 1e-9) {
        approach_heading = std::atan2(dy, dx);
    }

    if (params_.reverse) {
        approach_heading = normalizeAngle(approach_heading + kPi);
    }

    const double blend_distance = std::max(1.0, position_tolerance * 4.0);
    const double distance_fraction = clampUnit(distance_error / blend_distance);
    const double approach_fraction =
        clampUnit(params_.approachHeadingWeight) * distance_fraction;

    const double desired_heading =
        blendAngles(approach_heading, target_.theta, approach_fraction);

    const double heading_error =
        normalizeAngle(desired_heading - pose.theta);

    const double final_heading_error =
        normalizeAngle(target_.theta - pose.theta);

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
    }

    commandDriveAndTurn(linear_command, angular_command);

    bool within_tolerance =
        distance_error <= position_tolerance &&
        std::abs(final_heading_error) <= final_angle_tolerance;

    if (params_.settle.velocityTolerance >= 0.0) {
        within_tolerance =
            within_tolerance &&
            (translationalSpeed(velocity) <= params_.settle.velocityTolerance);
    }

    if (updateSettleTimer(within_tolerance, dt)) {
        finish(MotionStatus::SETTLED);
    }
}

void MoveToPose::updateStraightThenTurn(double dt) {
    const Pose2D pose = estimator_.getPose();
    const Twist2D velocity = estimator_.getVelocity();

    const double position_tolerance = std::max(0.0, params_.settle.positionTolerance);
    const double final_angle_tolerance = std::max(0.0, params_.maxFinalAngleError);

    const double dx = target_.x - pose.x;
    const double dy = target_.y - pose.y;
    const double distance_error = std::sqrt(dx * dx + dy * dy);
    const double final_heading_error =
        normalizeAngle(target_.theta - pose.theta);

    if (stage_ == StraightThenTurnStage::DRIVE_TO_POSITION) {
        if (distance_error <= position_tolerance) {
            stage_ = StraightThenTurnStage::TURN_TO_FINAL_HEADING;
            time_within_tolerance_seconds_ = 0.0;
            linear_controller_.reset();
            angular_controller_.reset();
            drivetrain_.stop();
            return;
        }

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

        linear_command = applyMinimumMagnitude(linear_command, params_.minSpeed);
        commandDriveAndTurn(linear_command, angular_command);

        time_within_tolerance_seconds_ = 0.0;
        return;
    }

    double angular_command = angular_controller_.updateFromError(final_heading_error, dt);
    angular_command = clampMagnitude(angular_command, params_.maxSpeed);

    if (std::abs(final_heading_error) > final_angle_tolerance) {
        angular_command = applyMinimumMagnitude(angular_command, params_.minSpeed);
    } else {
        angular_command = 0.0;
    }

    commandDriveAndTurn(0.0, angular_command);

    bool within_tolerance =
        distance_error <= position_tolerance &&
        std::abs(final_heading_error) <= final_angle_tolerance;

    if (params_.settle.velocityTolerance >= 0.0) {
        within_tolerance =
            within_tolerance &&
            (std::abs(velocity.omega) <= params_.settle.velocityTolerance);
    }

    if (updateSettleTimer(within_tolerance, dt)) {
        finish(MotionStatus::SETTLED);
    }
}

void MoveToPose::updateQuadraticBezier(double dt) {
    if (!path_.has_value()) {
        finish(MotionStatus::ERROR);
        return;
    }

    const Pose2D pose = estimator_.getPose();
    const Twist2D velocity = estimator_.getVelocity();

    const double position_tolerance = std::max(0.0, params_.settle.positionTolerance);
    const double final_angle_tolerance = std::max(0.0, params_.maxFinalAngleError);

    const double final_dx = target_.x - pose.x;
    const double final_dy = target_.y - pose.y;
    const double final_distance_error =
        std::sqrt(final_dx * final_dx + final_dy * final_dy);

    const double final_heading_error =
        normalizeAngle(target_.theta - pose.theta);

    const double search_window_t =
        std::max(0.0, params_.quadraticBezier.searchWindowT);
    const double lookahead_t =
        std::max(0.0, params_.quadraticBezier.lookaheadT);

    const double search_min = clampUnit(last_t_);
    const double search_max = clampUnit(last_t_ + search_window_t);

    double nearest_t =
        path_->findNearestT(pose.position(), search_min, search_max, 25);

    if (nearest_t < last_t_) {
        nearest_t = last_t_;
    }

    const double t_ref = clampUnit(nearest_t + lookahead_t);
    last_t_ = nearest_t;

    const Point2D reference_point = path_->samplePoint(t_ref);

    const double reference_dx = reference_point.x - pose.x;
    const double reference_dy = reference_point.y - pose.y;
    const double reference_distance_error =
        std::sqrt(reference_dx * reference_dx + reference_dy * reference_dy);

    double reference_heading = path_->sampleHeading(t_ref);

    // In reverse mode, the robot follows the path while facing opposite the
    // path tangent. Near the goal, heading regulation switches back to the
    // actual target final heading so settling still uses the requested pose.
    if (final_distance_error <=
        std::max(0.0, params_.quadraticBezier.finalHeadingSwitchDistance)) {
        reference_heading = target_.theta;
    } else if (params_.reverse) {
        reference_heading = normalizeAngle(reference_heading + kPi);
    }

    const double heading_error =
        normalizeAngle(reference_heading - pose.theta);

    double linear_command =
        linear_controller_.updateFromError(reference_distance_error, dt);
    double angular_command =
        angular_controller_.updateFromError(heading_error, dt);

    if (params_.reverse) {
        linear_command = -linear_command;
    }

    linear_command = clampMagnitude(linear_command, params_.maxSpeed);
    angular_command = clampMagnitude(angular_command, params_.maxSpeed);

    if (final_distance_error > position_tolerance && reference_distance_error > 1e-9) {
        linear_command = applyMinimumMagnitude(linear_command, params_.minSpeed);
    } else if (final_distance_error <= position_tolerance &&
               std::abs(final_heading_error) <= final_angle_tolerance) {
        linear_command = 0.0;
    }

    commandDriveAndTurn(linear_command, angular_command);

    bool within_tolerance =
        final_distance_error <= position_tolerance &&
        std::abs(final_heading_error) <= final_angle_tolerance;

    if (params_.settle.velocityTolerance >= 0.0) {
        within_tolerance =
            within_tolerance &&
            (translationalSpeed(velocity) <= params_.settle.velocityTolerance);
    }

    if (updateSettleTimer(within_tolerance, dt)) {
        finish(MotionStatus::SETTLED);
    }
}

}  // namespace studentlib