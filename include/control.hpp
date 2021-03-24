#include <chrono>
#include <vector>
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
  float pid_output;
  float dt;
  float error;
  float integral_error;
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

struct same_side_wheel_data {
  float front;
  float rear;
};

/* useful functions */
motor_data computeSkidSteerWheelSpeeds(robot_velocities target_velocities,
                                       robot_geometry robot_geometry);

robot_velocities limitAcceleration(robot_velocities target_velocities,
                                   robot_velocities measured_velocities,
                                   robot_velocities delta_v_limits, float dt);

same_side_wheel_data tractionScalePower(same_side_wheel_data motor_speeds,
                                        float gain);

robot_velocities computeVelocitiesFromWheelspeeds(
    motor_data wheel_speeds, robot_geometry robot_geometry);

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

  pid_outputs runControl(float target, float measured);

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
                            float max_motor_duty);

  void setAccelerationLimits(robot_velocities limits);
  robot_velocities getAccelerationLimits();

  void setOperatingMode(robot_motion_mode_t operating_mode);
  robot_motion_mode_t getOperatingMode();

  void setRobotGeometry(robot_geometry robot_geometry);
  robot_geometry getRobotGeometry();

  void setPidGains(pid_gains pid_gains);
  pid_gains getPidGains();

  void setTractionGain(float traction_control_gain);
  float getTractionGain();

  void setMotorMaxDuty(float max_motor_duty);
  float getMotorMaxDuty();

  void setFilterAlpha(float alpha);
  float getFilterAlpha();

  motor_data runMotionControl(robot_velocities velocity_targets,
                              motor_data current_duty_cycles,
                              motor_data current_motor_speeds);

 private:
  robot_motion_mode_t operating_mode_;
  robot_geometry robot_geometry_;

  PidController* pid_controller_left_;
  PidController* pid_controller_right_;
  pid_gains pid_gains_;

  float traction_control_gain_;
  float max_motor_duty_;
  float max_linear_acceleration_;
  float max_angular_acceleration_;
  float lpf_alpha_;
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
