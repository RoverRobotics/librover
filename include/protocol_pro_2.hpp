#pragma once
#include "protocol_base.hpp"
#include "vesc.hpp"
#include "utilities.hpp"

namespace RoverRobotics {
class Pro2ProtocolObject;

enum PRO_VESC_IDS{
  PRO_FRONT_LEFT = 0,
  PRO_FRONT_RIGHT = 1,
  PRO_BACK_LEFT = 2,
  PRO_BACK_RIGHT = 3
};

}

class RoverRobotics::Pro2ProtocolObject
    : public RoverRobotics::BaseProtocolObject {
 public:
  Pro2ProtocolObject(const char *device, std::string new_comm_type,
                     Control::robot_motion_mode_t robot_mode,
                     Control::pid_gains pid,
                     Control::angular_scaling_params angular_scale);
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
  void set_robot_velocity(double *controllarray) override;
  /*
   * @brief Unpack bytes from the robot
   * This is meant to use as a callback function when there are bytes available
   * to process
   * @param std::vector<uin32_t> Bytes stream from the robot
   * @return structure of statusData
   */
  void unpack_comm_response(std::vector<uint8_t>) override;
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
  void register_comm_base(const char *device) override;

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

  /*
   * @brief loads the persistent parameters from a non-volatile config file
   * 
   */
  void load_persistent_params();

  std::unique_ptr<Utilities::PersistentParams> persistent_params_;

  const std::string ROBOT_PARAM_PATH = strcat(std::getenv("HOME"), "/robot.config");
  

  /* metric units (meters) */
  Control::robot_geometry robot_geometry_ = {.intra_axle_distance = 0.2159,
                                             .wheel_base = 0.2794,
                                             .wheel_radius = 0.08255,
                                             .center_of_mass_x_offset = 0,
                                             .center_of_mass_y_offset = 0};
  const float MOTOR_RPM_TO_MPS_RATIO_ = 13749 / 1.26 / 0.72;
  const int MOTOR_NEUTRAL_ = 0;

  /* max: 1.0, min: 0.0  */
  const float MOTOR_MAX_ = .97;
  const float MOTOR_MIN_ = .02;
  float geometric_decay_ = .98;
  float left_trim_ = 1;
  float right_trim_ = 1;

  /* derivative of acceleration */
  const float LINEAR_JERK_LIMIT_ = 5;

  /* empirically measured */
  const float OPEN_LOOP_MAX_RPM_ = 600;

  /* limit to the trim that can be applied; more than this means a robot issue*/
  const float MAX_CURVATURE_CORRECTION_ = .15;

  int robotmode_num_ = Control::INDEPENDENT_WHEEL;

  const double CONTROL_LOOP_TIMEOUT_MS_ = 400;

  std::unique_ptr<Control::SkidRobotMotionController> skid_control_;
  std::unique_ptr<CommBase> comm_base_;
  std::string comm_type_;

  std::thread write_to_robot_thread_;
  std::thread motor_speed_update_thread_;
  std::mutex robotstatus_mutex_;

  /* main data structure */
  robotData robotstatus_;

  double motors_speeds_[4];
  double trimvalue_ = 0;
  
  bool estop_;

  Control::robot_motion_mode_t robot_mode_;
  Control::pid_gains pid_;
  Control::angular_scaling_params angular_scaling_params_;

  vesc::BridgedVescArray vescArray_;

};
