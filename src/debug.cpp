#include <stdio.h>

#include <memory>

#include "protocol_mini.hpp"
#include "protocol_pro_2.hpp"
#include "protocol_pro.hpp"
#include "protocol_zero_2.hpp"
#include "time.h"
using namespace RoverRobotics;

void print_status(RoverRobotics::robotData &robotdata) {
  std::cerr << "Robot Data " << std::endl
            << "motor 1 id " << robotdata.motor1.id << std::endl
            << "motor1.rpm " << robotdata.motor1.rpm << std::endl
            << "motor1.current " << robotdata.motor1.current << std::endl
            << "motor1.temp " << robotdata.motor1.temp << std::endl
            << "motor1.mos_temp " << robotdata.motor1.mos_temp << std::endl
            << "motor2.id " << robotdata.motor2.id << std::endl
            << "motor2.rpm " << robotdata.motor2.rpm << std::endl
            << "motor2.current " << robotdata.motor2.current << std::endl
            << "motor2.temp " << robotdata.motor2.temp << std::endl
            << "motor2.mos_temp " << robotdata.motor2.mos_temp << std::endl
            << "motor3.id " << robotdata.motor3.id << std::endl
            << "motor3.rpm " << robotdata.motor3.rpm << std::endl
            << "motor3.current " << robotdata.motor3.current << std::endl
            << "motor3.temp " << robotdata.motor3.temp << std::endl
            << "motor3.mos_temp " << robotdata.motor3.mos_temp << std::endl
            << "motor4.id " << robotdata.motor4.id << std::endl
            << "motor4.rpm " << robotdata.motor4.rpm << std::endl
            << "motor4.current " << robotdata.motor4.current << std::endl
            << "motor4.temp " << robotdata.motor4.temp << std::endl
            << "motor4.mos_temp " << robotdata.motor4.mos_temp << std::endl
            << "battery1.voltage " << robotdata.battery1.voltage << std::endl
            << "battery2.voltage " << robotdata.battery2.voltage << std::endl
            << "battery1.temp " << robotdata.battery1.temp << std::endl
            << "battery2.temp " << robotdata.battery2.temp << std::endl
            << "battery1.current " << robotdata.battery1.current << std::endl
            << "battery2.current " << robotdata.battery2.current << std::endl
            << "battery1.SOC " << robotdata.battery1.SOC << std::endl
            << "battery2.SOC " << robotdata.battery2.SOC << std::endl
            << "battery1.fault_flag " << robotdata.battery1.fault_flag
            << std::endl
            << "battery2.fault_flag " << robotdata.battery2.fault_flag
            << std::endl
            << "robot_guid " << robotdata.robot_guid << std::endl
            << "robot_firmware " << robotdata.robot_firmware << std::endl
            << "robot_fault_flag " << robotdata.robot_fault_flag << std::endl
            << "robot_fan_speed " << robotdata.robot_fan_speed << std::endl
            << "robot_speed_limit " << robotdata.robot_speed_limit << std::endl
            << "motor3_angle " << robotdata.motor3_angle << std::endl
            << "motor3_sensor1 " << robotdata.motor3_sensor1 << std::endl
            << "motor3_sensor2 " << robotdata.motor3_sensor2 << std::endl
            << "linear_vel " << robotdata.linear_vel << std::endl
            << "angular_vel " << robotdata.angular_vel << std::endl
            << "cmd_linear_vel " << robotdata.cmd_linear_vel << std::endl
            << "cmd_angular_vel " << robotdata.cmd_angular_vel << std::endl
	    << "Firmware" << robotdata.robot_firmware << std::endl;
}

int main() {
  // Control::pid_gains testgains_ = {0.0009, 0, 0.00007};
  Control::pid_gains testgains_ = {0, 0, 0};

  Control::robot_motion_mode_t robot_mode = Control::INDEPENDENT_WHEEL;
  Control::angular_scaling_params angular_scaling_params_ = {0, 1, 0, 1, 1};
  // std::unique_ptr<BaseProtocolObject> robot_ =
  //     std::make_unique<MiniProtocolObject>(
  //         "can0", "can", robot_mode, testgains_,
  //         angular_scaling_params_);
  // std::make_unique<Pro2ProtocolObject>(
  //       "can0", "can", robot_mode, testgains_,
  //       angular_scaling_params_);
  std::unique_ptr<BaseProtocolObject> robot_ =
      std::make_unique<ProProtocolObject>("/dev/rover-pro", "serial",
                                            robot_mode, testgains_);
  //robot_->cycle_robot_mode();

  while (true) {
    auto status = robot_->status_request();
    // std::cout << status.angular_vel << std::endl;

    // auto connected = robot_->is_connected();
    // std::cout << "connected:  " << connected << std::endl;

    auto info = robot_->info_request();
    print_status(info);
    //double controlarray[2] = {1, 0};
    //robot_->set_robot_velocity(controlarray);
    // robot_->cycle_robot_mode();

    // robot_->send_estop(true);

    // robot_->update_drivetrim(0.01);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}
