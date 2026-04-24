#include "studentlib/motion/motion_manager.hpp"

#include <memory>

#include "studentlib/motion/drive_to_point.hpp"
#include "studentlib/motion/move_to_pose.hpp"
#include "studentlib/motion/turn_to_heading.hpp"
#include "studentlib/motion/turn_to_point.hpp"

namespace studentlib {

MotionManager::MotionManager(Drivetrain& drivetrain, StateEstimator& estimator)
    : drivetrain_(drivetrain),
      estimator_(estimator) {}

void MotionManager::update(double dt) {
    if (!active_command_) {
        return;
    }

    if (!active_command_->isFinished()) {
        active_command_->update(dt);
    }
}

void MotionManager::cancel() {
    if (!active_command_) {
        return;
    }

    active_command_->cancel();
}

MotionStatus MotionManager::getStatus() const {
    if (!active_command_) {
        return MotionStatus::IDLE;
    }

    return active_command_->getStatus();
}

bool MotionManager::isBusy() const {
    return active_command_ && !active_command_->isFinished();
}

bool MotionManager::isFinished() const {
    return !active_command_ || active_command_->isFinished();
}

void MotionManager::startTurnToHeading(
    double targetHeadingRadians,
    const TurnToHeadingParams& params) {

    startCommand(std::make_unique<TurnToHeading>(
        drivetrain_,
        estimator_,
        targetHeadingRadians,
        params));
}

void MotionManager::startTurnToPoint(
    const Point2D& target,
    const TurnToPointParams& params) {

    startCommand(std::make_unique<TurnToPoint>(
        drivetrain_,
        estimator_,
        target,
        params));
}

void MotionManager::startDriveToPoint(
    const Point2D& target,
    const DriveToPointParams& params) {

    startCommand(std::make_unique<DriveToPoint>(
        drivetrain_,
        estimator_,
        target,
        params));
}

void MotionManager::startMoveToPose(
    const Pose2D& target,
    const MoveToPoseParams& params) {

    startCommand(std::make_unique<MoveToPose>(
        drivetrain_,
        estimator_,
        target,
        params));
}

void MotionManager::startCommand(std::unique_ptr<MotionCommand> command) {
    if (active_command_) {
        active_command_->cancel();
    }

    active_command_ = std::move(command);

    if (active_command_) {
        active_command_->start();
    }
}

}  // namespace studentlib