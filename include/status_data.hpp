#include <chrono>
#pragma once
namespace RoverRobotics {
struct motorData {
  signed short int id;
  float rpm;
  signed short int current;
  signed short int temp;
  signed short int mos_temp;
  };

struct batteryData {
  unsigned short int voltage;
  signed short int temp;
  unsigned short int current;
  bool SOC;
  unsigned short int fault_flag;
  };

struct robotData {
  // Motor Infos
  motorData motor1;
  motorData motor2;
  motorData motor3;
  motorData motor4;

  // Battery Infos
  batteryData battery1;
  batteryData battery2;

  // Robot FEEDBACK Infos
  unsigned short int robot_guid;
  unsigned short int robot_firmware;
  unsigned short int robot_fault_flag;
  unsigned short int robot_fan_speed;
  unsigned short int robot_speed_limit;

  // Flipper Infos
  unsigned short int motor3_angle;
  unsigned short int motor3_sensor1;
  unsigned short int motor3_sensor2;

  // Robot Info
  double linear_vel;
  double angular_vel;

  // Velocity Info
  double cmd_linear_vel;
  double cmd_angular_vel;
  std::chrono::milliseconds cmd_ts;

  // estop info
  bool estop;
};
}  // namespace RoverRobotics