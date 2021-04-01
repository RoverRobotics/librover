#include "debug.hpp"
#include <stdio.h>

#include <memory>

#include "time.h"
namespace RoverRobotics {
int Rover() {
  Control::pid_gains testgains_ = {0, 0, 0};
  Control::robot_motion_mode_t robot_mode = Control::OPEN_LOOP;
  robot_ =
      std::make_unique<Pro2ProtocolObject>("can0", "can", robot_mode, testgains_);
}
}  // namespace RoverRobotics
int main() { int rover = RoverRobotics::Rover(); 
while (true){
  continue;
}}
