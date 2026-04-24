#pragma once

#include <optional>

#include "studentlib/control/pid_controller.hpp"
#include "studentlib/drivetrain/drivetrain.hpp"
#include "studentlib/estimation/state_estimator.hpp"
#include "studentlib/math/pose.hpp"
#include "studentlib/motion/motion_command.hpp"
#include "studentlib/motion/motion_params.hpp"
#include "studentlib/motion/quadratic_bezier_path.hpp"

namespace studentlib {

class MoveToPose : public MotionCommand {
public:
    explicit MoveToPose(
        Drivetrain& drivetrain,
        StateEstimator& estimator,
        const Pose2D& target,
        const MoveToPoseParams& params = {});

    void start() override;
    void update(double dt) override;
    bool isFinished() const override;
    MotionStatus getStatus() const override;
    void cancel() override;

private:
    enum class StraightThenTurnStage {
        DRIVE_TO_POSITION,
        TURN_TO_FINAL_HEADING
    };

    Drivetrain& drivetrain_;
    StateEstimator& estimator_;
    PIDController linear_controller_;
    PIDController angular_controller_;
    Pose2D target_{};
    MoveToPoseParams params_{};

    MotionStatus status_{MotionStatus::IDLE};
    bool started_{false};
    double elapsed_time_seconds_{0.0};
    double time_within_tolerance_seconds_{0.0};

    StraightThenTurnStage stage_{StraightThenTurnStage::DRIVE_TO_POSITION};

    std::optional<QuadraticBezierPath> path_{};
    double last_t_{0.0};

    void finish(MotionStatus status);
    void commandDriveAndTurn(double linearCommand, double angularCommand);
    bool updateSettleTimer(bool withinTolerance, double dt);

    void updatePoseRegulator(double dt);
    void updateStraightThenTurn(double dt);
    void updateQuadraticBezier(double dt);
};

}  // namespace studentlib