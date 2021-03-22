#include <chrono>
#include <vector>

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

struct pid_output_limits {
  float posmax;
  float negmax;
};

struct same_side_wheel_data {
  float front;
  float rear;
};

/* useful functions */
motor_data computeSkidSteerWheelSpeeds(robot_velocities target_velocities,
                                       robot_geometry robot_geometry);

robot_velocities limitVelocityChange(robot_velocities target_velocities,
                                     robot_velocities measured_velocities,
                                     robot_velocities delta_v_limits);

same_side_wheel_data tractionScalePower(same_side_wheel_data motor_speeds,
                                        float gain);

}  // namespace Control

class Control::PidController {
 public:
  /* constructors */
  PidController(pid_gains pid_gains);
  PidController(pid_gains pid_gains, pid_output_limits pid_output_limits);

  void setGains(pid_gains pid_gains);
  pid_gains getGains();

  void setOutputLimits(pid_output_limits pid_output_limits);
  pid_output_limits getOutputLimits();

  void setIntegralErrorLimit(float error_limit);
  float getIntegralErrorLimit();

  float runControl(float target, float measured);

 private:
  float kp_;
  float ki_;
  float kd_;
  float integral_error_;
  float integral_error_limit_;
  float pos_max_output_;
  float neg_max_output_;
  std::chrono::milliseconds time_last_;
};

class Control::SkidRobotMotionController {
 public:
  /* constructors */
  SkidRobotMotionController();
  SkidRobotMotionController(robot_motion_mode_t operating_mode,
                            robot_geometry robot_geometry, pid_gains pid_gains,
                            pid_output_limits pid_limits);

  void enableVelocityChangeLimiting(robot_velocities limits);

  void changeOperatingMode(robot_motion_mode_t operating_mode);

  void setRobotGeometry(robot_geometry robot_geometry);

  void setPidGains(pid_gains pid_gains);

  void setTractionGain(float traction_control_gain);

  void setMotorMaxDuty(float max_motor_duty);

  void setFilterAlpha(float alpha);

  motor_data runMotionControl(robot_velocities velocity_targets,
                              motor_data current_duty_cycles,
                              motor_data current_motor_speeds);

 private:
  PidController pid_controller_left_;
  PidController pid_controller_right_;

  pid_gains pid_gains_left_;
  pid_gains pid_gains_right_;

  float traction_control_gain_;

  float max_motor_duty_;
  
};

class Control::AlphaBetaFilter {
 public:
  /* constructors */
  AlphaBetaFilter(float alpha);

  float update(float new_value);

 private:
  float running_sum_;
  
};




