#pragma once

#include <memory>

#include "studentlib/drivetrain/drivetrain.hpp"
#include "studentlib/estimation/state_estimator.hpp"
#include "studentlib/math/point.hpp"
#include "studentlib/math/pose.hpp"
#include "studentlib/motion/motion_command.hpp"
#include "studentlib/motion/motion_params.hpp"
#include "studentlib/motion/motion_status.hpp"

namespace studentlib {

class MotionManager {
public:
    explicit MotionManager(Drivetrain& drivetrain, StateEstimator& estimator);

    void update(double dt);
    void cancel();

    MotionStatus getStatus() const;
    bool isBusy() const;
    bool isFinished() const;

    void startTurnToHeading(
        double targetHeadingRadians,
        const TurnToHeadingParams& params = {});

    void startTurnToPoint(
        const Point2D& target,
        const TurnToPointParams& params = {});

    void startDriveToPoint(
        const Point2D& target,
        const DriveToPointParams& params = {});

    void startMoveToPose(
        const Pose2D& target,
        const MoveToPoseParams& params = {});

private:
    Drivetrain& drivetrain_;
    StateEstimator& estimator_;
    std::unique_ptr<MotionCommand> active_command_{};

    void startCommand(std::unique_ptr<MotionCommand> command);
};

}  // namespace studentlib