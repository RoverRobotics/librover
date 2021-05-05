#include <stdio.h>

#include <memory>

#include "librover/protocol_pro_2.hpp"
#include "time.h"
using namespace RoverRobotics;

int main() {

  Control::pid_gains testgains_ = {0, 0, 0};
  Control::robot_motion_mode_t robot_mode = Control::OPEN_LOOP;
  Control::angular_scaling_params angular_scaling_params_ = {0, 0, 0, 0, 0};
  std::unique_ptr<BaseProtocolObject> robot_ = std::make_unique<Pro2ProtocolObject>(
        "can0", "can", robot_mode, testgains_,
        angular_scaling_params_);
  
  while (true) {
    auto status = robot_->status_request();
    //std::cout << status.angular_vel << std::endl;

    auto connected = robot_->is_connected();
    //std::cout << "connected: " << connected << std::endl;

    auto info = robot_->info_request();
    //std::cout << info.angular_vel << std::endl; 

    robot_->cycle_robot_mode();

    robot_->send_estop(true);

    robot_->update_drivetrim(0.01);
    //std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}
