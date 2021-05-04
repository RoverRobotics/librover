#include <stdio.h>

#include <memory>

#include "librover/protocol_pro_2.hpp"
#include "time.h"
namespace RoverRobotics {
int Rover() {
  Control::pid_gains testgains_ = {0, 0, 0};
  Control::robot_motion_mode_t robot_mode = Control::OPEN_LOOP;
  Control::angular_scaling_params angular_scaling_params_ = {0, 0, 0, 0, 0};
  std::unique_ptr<BaseProtocolObject> robot_ = std::make_unique<Pro2ProtocolObject>(
        "can0", "can", robot_mode, testgains_,
        angular_scaling_params_);
}
}  // namespace RoverRobotics
int main() {
  int rover = RoverRobotics::Rover();
  while (true) {
    continue;
  }
}
