#pragma once

#include "studentlib/control/pid_controller.hpp"
#include "studentlib/drivetrain/drivetrain.hpp"
#include "studentlib/estimation/state_estimator.hpp"
#include "studentlib/motion/motion_command.hpp"
#include "studentlib/motion/motion_params.hpp"

namespace studentlib {

class TurnToHeading : public MotionCommand {
public:
    explicit TurnToHeading(
        Drivetrain& drivetrain,
        StateEstimator& estimator,
        double targetHeadingRadians,
        const TurnToHeadingParams& params = {});

    void start() override;
    void update(double dt) override;
    bool isFinished() const override;
    MotionStatus getStatus() const override;
    void cancel() override;

private:
    Drivetrain& drivetrain_;
    StateEstimator& estimator_;
    PIDController angular_controller_;
    double target_heading_radians_{0.0};
    TurnToHeadingParams params_{};

    MotionStatus status_{MotionStatus::IDLE};
    bool started_{false};
    double elapsed_time_seconds_{0.0};
    double time_within_tolerance_seconds_{0.0};
};

}  // namespace studentlib