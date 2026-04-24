#pragma once

#include <optional>

#include "studentlib/estimation/motion_delta.hpp"
#include "studentlib/estimation/sensor_snapshot.hpp"
#include "studentlib/estimation/state_estimator.hpp"
#include "studentlib/hardware/sensor_suite.hpp"
#include "studentlib/model/robot_model.hpp"

namespace studentlib {

class OdometryEstimator : public StateEstimator {
public:
    explicit OdometryEstimator(SensorSuite& sensors, const RobotModel& robotModel);

    Pose2D getPose() const override;
    void setPose(const Pose2D& pose) override;
    Twist2D getVelocity() const override;
    void reset() override;
    void step(double dt) override;

private:
    SensorSuite& sensors_;
    const RobotModel& robot_model_;

    Pose2D pose_{};
    Twist2D velocity_{};

    std::optional<SensorSnapshot> previous_snapshot_{};

    SensorSnapshot captureSnapshot() const;
    MotionDelta computeMotionDelta(const SensorSnapshot& current) const;
};

}  // namespace studentlib