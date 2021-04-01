#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#ifdef DEBUG
#include <ctime>
#include <sstream>
#endif

#define RPM_TO_RADS_SEC 0.10472

namespace Control {

/* classes */
class PidController;
class SkidRobotMotionController;
class AlphaBetaFilter;

/* datatypes */
typedef enum {
  OPEN_LOOP,
  TRACTION_CONTROL,
  INDEPENDENT_WHEEL
} robot_motion_mode_t;

struct robot_velocities {
  float linear_velocity;
  float angular_velocity;
};

struct motor_data {
  float fl;
  float fr;
  float rl;
  float rr;
};

struct robot_geometry {
  float intra_axle_distance;
  float wheel_base;
  float wheel_radius;
  float center_of_mass_x_offset;
  float center_of_mass_y_offset;
};

struct pid_gains {
  float kp;
  float ki;
  float kd;
};

struct pid_outputs {
  std::string name;
  double time;
  float dt;
  float pid_output;
  float error;
  float integral_error;
  float delta_error;
  float target_value;
  float measured_value;
  float kp;
  float ki;
  float kd;
};

struct pid_output_limits {
  float posmax;
  float negmax;
};

/* useful functions */
motor_data computeSkidSteerWheelSpeeds(robot_velocities target_velocities,
                                       robot_geometry robot_geometry);

robot_velocities limitAcceleration(robot_velocities target_velocities,
                                   robot_velocities measured_velocities,
                                   robot_velocities delta_v_limits, float dt);

robot_velocities computeVelocitiesFromWheelspeeds(
    motor_data wheel_speeds, robot_geometry robot_geometry);

}  // namespace Control

class Control::PidController {
 public:
  /* constructors */
  PidController(pid_gains pid_gains, std::string name);
  PidController(pid_gains pid_gains, pid_output_limits pid_output_limits,
                std::string name);

  void setGains(pid_gains pid_gains);
  pid_gains getGains();

  void setOutputLimits(pid_output_limits pid_output_limits);
  pid_output_limits getOutputLimits();

  void setIntegralErrorLimit(float error_limit);
  float getIntegralErrorLimit();

  pid_outputs runControl(float target, float measured);

  void writePidDataToCsv(std::ofstream &log_file, pid_outputs data);

 private:
  std::string name_;
  float kp_;
  float ki_;
  float kd_;
  float integral_error_;
  float integral_error_limit_;
  float previous_error_;
  float pos_max_output_;
  float neg_max_output_;
  std::chrono::steady_clock::time_point time_last_;
  std::chrono::steady_clock::time_point time_origin_;
};

class Control::SkidRobotMotionController {
 public:
  /* constructors */
  SkidRobotMotionController(float max_motor_duty = 0.95,
                            float min_motor_duty = 0.03, float left_trim = 1.0,
                            float right_trim = 1.0,
                            float open_loop_max_motor_rpm = 600);
  SkidRobotMotionController(robot_motion_mode_t operating_mode,
                            robot_geometry robot_geometry, pid_gains pid_gains,
                            float max_motor_duty = 0.95,
                            float min_motor_duty = 0.03, float left_trim = 1.0,
                            float right_trim = 1.0,
                            float geometric_decay = 0.99);

  void setAccelerationLimits(robot_velocities limits);
  robot_velocities getAccelerationLimits();

  void setOperatingMode(robot_motion_mode_t operating_mode);
  robot_motion_mode_t getOperatingMode();

  void setRobotGeometry(robot_geometry robot_geometry);
  robot_geometry getRobotGeometry();

  void setPidGains(pid_gains pid_gains);
  pid_gains getPidGains();

  void setMotorMaxDuty(float max_motor_duty);
  float getMotorMaxDuty();

  void setOutputDecay(float geometric_decay);
  float getOutputDecay();

  motor_data runMotionControl(robot_velocities velocity_targets,
                              motor_data current_duty_cycles,
                              motor_data current_motor_speeds);

  robot_velocities getMeasuredVelocities(motor_data current_motor_speeds);

 private:
  std::string log_folder_path_;
#ifdef DEBUG
  std::ofstream log_file_;
#endif

  robot_motion_mode_t operating_mode_;
  robot_geometry robot_geometry_;

  std::unique_ptr<PidController> pid_controller_left_;
  std::unique_ptr<PidController> pid_controller_right_;

  std::unique_ptr<PidController> pid_controller_fl_;
  std::unique_ptr<PidController> pid_controller_fr_;
  std::unique_ptr<PidController> pid_controller_rl_;
  std::unique_ptr<PidController> pid_controller_rr_;

  pid_gains pid_gains_;
  robot_velocities measured_velocities_;

  float open_loop_max_motor_rpm_;

  float max_motor_duty_;
  float min_motor_duty_;

  float left_trim_value_;
  float right_trim_value_;

  float max_linear_acceleration_;
  float max_angular_acceleration_;

  float geometric_decay_;

  std::chrono::steady_clock::time_point time_last_;
  std::chrono::steady_clock::time_point time_origin_;

  motor_data duty_cycles_;

  motor_data computeMotorCommandsTc_(motor_data target_wheel_speeds,
                                     motor_data current_motor_speeds);

  motor_data clipDutyCycles_(motor_data proposed_duties);

  motor_data computeTorqueDistribution_(motor_data current_motor_speeds,
                                        motor_data power_proposals);
};

// #TODO: implement if needed
class Control::AlphaBetaFilter {
 public:
  /* constructors */
  AlphaBetaFilter(float alpha);

  float update(float new_value);

 private:
  float running_sum_;
};
