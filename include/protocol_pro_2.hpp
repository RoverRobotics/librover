#pragma once
#include "protocol_base.hpp"

namespace RoverRobotics {
class Pro2ProtocolObject;
}
class RoverRobotics::Pro2ProtocolObject
    : public RoverRobotics::BaseProtocolObject {
 public:
  Pro2ProtocolObject(const char* device, std::string new_comm_type,
                     Control::robot_motion_mode_t robot_mode, Control::pid_gains pid);
  /*
   * @brief Trim Robot Velocity
   * Modify robot velocity differential (between the left side/right side) with
   * the input parameter. Useful to compensate if the robot tends to drift
   * either left or right while commanded to drive straight.
   * @param double of velocity offset
   */
  void update_drivetrim(double) override;
  /*
   * @brief Handle Estop Event
   * Send an estop event to the robot
   * @param bool accept a estop state
   */
  void send_estop(bool) override;
  /*
   * @brief Request Robot Status
   * @return structure of statusData
   */
  robotData status_request() override;
  /*
   * @brief Request Robot Unique Infomation
   * @return structure of statusData
   */
  robotData info_request() override;
  /*
   * @brief Set Robot velocity
   * Set Robot velocity: IF robot_mode_ TRUE, this function will attempt a
   * speed PID loop which uses all the available sensor data (wheels, IMUs, etc)
   * from the robot to produce the commanded velocity as best as possible. IF
   * robot_mode_ FALSE, this function simply translates the commanded
   * velocities into motor duty cycles and there is no expectation that the
   * commanded velocities will be realized by the robot. In robot_mode_ FALSE
   * mode, motor power is roughly proportional to commanded velocity.
   * @param controllarray an double array of control in m/s
   */
  void set_robot_velocity(double* controllarray) override;
  /*
   * @brief Unpack bytes from the robot
   * This is meant to use as a callback function when there are bytes available
   * to process
   * @param std::vector<uin32_t> Bytes stream from the robot
   * @return structure of statusData
   */
  void unpack_comm_response(std::vector<uint32_t>) override;
  /*
   * @brief Check if Communication still exist
   * @return bool
   */
  bool is_connected() override;
  /*
   * @brief Cycle through robot supported modes
   * @return int of the current mode enum
   */
  int cycle_robot_mode() override;
  /*
   * @brief Attempt to make connection to robot via device
   * @param device is the address of the device (ttyUSB0 , can0, ttyACM0, etc)
   */
  void register_comm_base(const char* device) override;
  

 private:
  /*
   * @brief Thread Driven function that will send commands to the robot at set
   * interval
   * @param sleeptime sleep time between each cycle
   * @param datalist list of data to request
   */
  void send_command(int sleeptime);
  /*
   * @brief Thread Driven function update the robot motors using pid
   * @param sleeptime sleep time between each cycle
   */
  void motors_control_loop(int sleeptime);
  const float MOTOR_RPM_TO_MPS_RATIO_ = 13749 / 1.26 / 0.72;
  const int MOTOR_NEUTRAL_ = 0;
  const int MOTOR_MAX_ = .95;
  const int MOTOR_MIN_ = .03;
  // Parameterize?
  const Control::robot_geometry robot_geometry_ = {0.205, 0.265, .09, 0, 0};
  const float geometric_decay_ = .99;
  float left_trim_ = 1;
  float right_trim_ = .99;



  //motors control 
  Control::SkidRobotMotionController skid_control_;
  const double CONTROL_LOOP_TIMEOUT_MS_ = 100;
  std::unique_ptr<CommBase> comm_base_;
  std::string comm_type_;

  std::mutex robotstatus_mutex_;
  robotData robotstatus_;
  double motors_speeds_[4];
  double trimvalue_;
  std::thread write_to_robot_thread_;
  std::thread motor_speed_update_thread_;
  bool estop_;
  // Motor PID variables
  Control::robot_motion_mode_t robot_mode_;
  Control::pid_gains pid_;

  enum robot_motors {
    FRONT_LEFT_MOTOR,
    FRONT_RIGHT_MOTOR,
    BACK_LEFT_MOTOR,
    BACK_RIGHT_MOTOR
  };
};
