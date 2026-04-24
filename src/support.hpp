#pragma once

#include "studentlib/robot/robot.hpp"
#include "studentlib/motion/motion_status.hpp"

namespace demo {

studentlib::Robot& robot();

void initializeRobot();
void printPoseAndStatus(const char* label);

double joystickToNormalized(double joystickValue);

}  // namespace demo