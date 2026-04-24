#pragma once

#include "studentlib/control/pid_controller.hpp"
#include "studentlib/drivetrain/drivetrain.hpp"
#include "studentlib/estimation/state_estimator.hpp"
#include "studentlib/math/point.hpp"
#include "studentlib/motion/motion_command.hpp"
#include "studentlib/motion/motion_params.hpp"

namespace studentlib {

class DriveToPoint : public MotionCommand {
public:
    explicit DriveToPoint(
        Drivetrain& drivetrain,
        StateEstimator& estimator,
        const Point2D& target,
        const DriveToPointParams& params = {});

    void start() override;
    void update(double dt) override;
    bool isFinished() const override;
    MotionStatus getStatus() const override;
    void cancel() override;

private:
    Drivetrain& drivetrain_;
    StateEstimator& estimator_;
    PIDController linear_controller_;
    PIDController angular_controller_;
    Point2D target_{};
    DriveToPointParams params_{};

    MotionStatus status_{MotionStatus::IDLE};
    bool started_{false};
    double elapsed_time_seconds_{0.0};
    double time_within_tolerance_seconds_{0.0};
};

}  // namespace studentlib