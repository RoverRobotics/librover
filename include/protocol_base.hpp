#pragma once

#include "comm_base.hpp"
#include "comm_serial.hpp"

namespace RoverRobotics {
class BaseProtocolObject;
}
class RoverRobotics::BaseProtocolObject {
 public:
  /*
   * @brief Trim Robot Velocity
   * Modify robot velocity offset with the input parameter
   * @param double of velocity offset
   */
  virtual void update_drivetrim(double) = 0;
  /*
   * @brief Handle Estop Event
   * Send an estop event to the robot
   * @param bool accept a estop state
   */
  virtual void send_estop(bool) = 0;
  /*
   * @brief Set Robot velocity
   * Set Robot velocity: IF closed_loop_ TRUE, this function will attempt a speed PID loop 
   * which uses all the available sensor data (wheels, IMUs, etc) from the robot to produce
   * the commanded velocity as best as possible. IF closed_loop_ FALSE, this function simply
   * translates the commanded velocities into motor duty cycles and there is no expectation that
   * the commanded velocities will be realized by the robot. In closed_loop_ FALSE mode, motor
   * power is roughly proportional to commanded velocity.
   * @param controllarray an double array of control in m/s
   */
  virtual void set_robot_velocity(double* controllarray) = 0;
  /*
   * @brief Request Robot Status
   * @return structure of statusData
   */
  virtual robotData status_request() = 0;
  /*
   * @brief Request Robot Unique Infomation
   * @return structure of statusData
   */
  virtual robotData info_request() = 0;
  /*
   * @brief Unpack bytes from the robot
   * This is meant to use as a callback function when there are bytes avaible to process
   * @param std::vector<uin32_t> Bytes stream from the robot
   * @return structure of statusData
   */
  virtual void unpack_comm_response(std::vector<uint32_t>) = 0;
  /*
   * @brief Check if Communication still exist
   * @return bool
   */
  virtual bool is_connected() = 0;
  /*
   * @brief Attempt to make connection to robot via device
   * @param device is the address of the device (ttyUSB0 , can0, ttyACM0)
   */
  virtual void register_comm_base(const char* device) = 0;
};
