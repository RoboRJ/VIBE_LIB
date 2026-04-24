#pragma once

#include "studentlib/estimation/mcl/mcl_config.hpp"
#include "studentlib/estimation/sensor_snapshot.hpp"
#include "studentlib/math/pose.hpp"
#include "studentlib/model/field_map.hpp"
#include "studentlib/model/robot_model.hpp"

namespace studentlib {

class SensorModel {
public:
    explicit SensorModel(const MCLConfig& config);

    double likelihood(
        const Pose2D& particlePose,
        const SensorSnapshot& snapshot,
        const RobotModel& robotModel,
        const FieldMap& fieldMap) const;

private:
    MCLConfig config_;

    double gaussianLogLikelihood(double error, double standardDeviation) const;
};

}  // namespace studentlib