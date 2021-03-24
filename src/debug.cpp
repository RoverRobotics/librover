#include "debug.hpp"

#include <stdio.h>

#include <memory>

#include "time.h"
namespace RoverRobotics {
int Rover() {
  PidGains pidGains_ = {0, 0, 0};
  robot_ =
      std::make_unique<Pro2ProtocolObject>("can0", "can", false, pidGains_);
}
}  // namespace RoverRobotics
int main() { int rover = RoverRobotics::Rover(); 
while (true){
  continue;
}}
