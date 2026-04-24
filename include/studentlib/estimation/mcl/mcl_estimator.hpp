#pragma once

#include <optional>
#include <vector>

#include "studentlib/estimation/mcl/mcl_config.hpp"
#include "studentlib/estimation/mcl/motion_model.hpp"
#include "studentlib/estimation/mcl/particle.hpp"
#include "studentlib/estimation/mcl/resampler.hpp"
#include "studentlib/estimation/mcl/sensor_model.hpp"
#include "studentlib/estimation/motion_delta.hpp"
#include "studentlib/estimation/sensor_snapshot.hpp"
#include "studentlib/estimation/state_estimator.hpp"
#include "studentlib/hardware/sensor_suite.hpp"
#include "studentlib/model/field_map.hpp"
#include "studentlib/model/robot_model.hpp"

namespace studentlib {

class MCLEstimator : public StateEstimator {
public:
    explicit MCLEstimator(
        SensorSuite& sensors,
        const RobotModel& robotModel,
        const FieldMap& fieldMap,
        const MCLConfig& config = {});

    Pose2D getPose() const override;
    void setPose(const Pose2D& pose) override;
    Twist2D getVelocity() const override;
    void reset() override;
    void step(double dt) override;

private:
    SensorSuite& sensors_;
    const RobotModel& robot_model_;
    const FieldMap& field_map_;

    MCLConfig config_;
    MotionModel motion_model_;
    SensorModel sensor_model_;
    Resampler resampler_;

    std::vector<Particle> particles_{};
    Pose2D extracted_pose_{};
    Twist2D velocity_{};
    std::optional<SensorSnapshot> previous_snapshot_{};

    void initializeParticlesAtPose(const Pose2D& pose);
    SensorSnapshot captureSnapshot() const;
    MotionDelta computeMotionDelta(
        const SensorSnapshot& previous,
        const SensorSnapshot& current) const;
    Pose2D extractPose() const;
};

}  // namespace studentlib