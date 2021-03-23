#include "control.hpp"

#include <limits>

namespace Control {
PidController::PidController(struct pid_gains pid_gains)
    : /* defaults */
      integral_error_(0),
      integral_error_limit_(std::numeric_limits<float>::max()),
      pos_max_output_(std::numeric_limits<float>::max()),
      neg_max_output_(std::numeric_limits<float>::lowest()),
      time_last_(std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())) {
  kp_ = pid_gains.kp;
  kd_ = pid_gains.kd;
  ki_ = pid_gains.ki;
};

PidController::PidController(struct pid_gains pid_gains,
                             pid_output_limits pid_output_limits)
    : /* defaults */
      integral_error_(0),
      integral_error_limit_(std::numeric_limits<float>::max()),
      time_last_(std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())) {
  kp_ = pid_gains.kp;
  kd_ = pid_gains.kd;
  ki_ = pid_gains.ki;
  pos_max_output_ = pid_output_limits.posmax;
  neg_max_output_ = pid_output_limits.negmax;
};

void PidController::setGains(struct pid_gains pid_gains) {
  kp_ = pid_gains.kp;
  kd_ = pid_gains.kd;
  ki_ = pid_gains.ki;
};

pid_gains PidController::getGains() {
  pid_gains pid_gains;
  pid_gains.kp = kp_;
  pid_gains.ki = ki_;
  pid_gains.kd = kd_;
  return pid_gains;
}

void PidController::setOutputLimits(pid_output_limits pid_output_limits) {
  pos_max_output_ = pid_output_limits.posmax;
  neg_max_output_ = pid_output_limits.negmax;
}

pid_output_limits PidController::getOutputLimits() {
  pid_output_limits returnstruct;
  returnstruct.posmax = pos_max_output_;
  returnstruct.negmax = neg_max_output_;
  return returnstruct;
}

void PidController::setIntegralErrorLimit(float error_limit) {
  integral_error_limit_ = error_limit;
}

float PidController::getIntegralErrorLimit() { return integral_error_limit_; }

pid_outputs PidController::runControl(float target, float measured) {
  /* current time */
  std::chrono::milliseconds time_now =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());

  /* delta time (S) */
  float delta_time = (time_now - time_last_).count() / 1000;

  /* update time bookkeeping */
  time_last_ = time_now;

#ifdef DEBUG
  std::cerr << 'dt ' << delta_time << std::endl;
#endif

  /* error */
  float error = target - measured;
#ifdef DEBUG
  std::cerr << 'error ' << error << std::endl;
#endif

  /* integrate */
  integral_error_ += error;

  /* clip integral error */
  if (integral_error_ > integral_error_limit_) {
    integral_error_ = integral_error_limit_;
  }
  if (integral_error_ < -integral_error_limit_) {
    integral_error_ = -integral_error_limit_;
  }
#ifdef DEBUG
  std::cerr << 'int_error ' << integral_error_ << std::endl;
#endif

  /* P I D terms */
  float p = error * kp_;
  float i = integral_error_ * ki_;
  float d = delta_time * kd_;

#ifdef DEBUG
  std::cerr << 'p ' << p << std::endl;
  std::cerr << 'i ' << i << std::endl;
  std::cerr << 'd ' << d << std::endl;
#endif

  /* compute output */
  float output = p + i + d;

  /* clip output */
  if (output > pos_max_output_) {
    output = pos_max_output_;
  }
  if (output < neg_max_output_) {
    output = neg_max_output_;
  }

#ifdef DEBUG
  std::cerr << 'output ' << output << std::endl;
#endif

  pid_outputs returnstruct;
  returnstruct.pid_output = output;
  returnstruct.dt = delta_time;
  returnstruct.error = error;
  returnstruct.integral_error = integral_error_;
  returnstruct.target_value = target;
  returnstruct.measured_value = measured;
  returnstruct.kp = kp_;
  returnstruct.ki = ki_;
  returnstruct.kd = kd_;

  return returnstruct;
}

SkidRobotMotionController::SkidRobotMotionController()
    : operating_mode_(OPEN_LOOP),
      traction_control_gain_(1),
      max_motor_duty_(100) {}

SkidRobotMotionController::SkidRobotMotionController(
    robot_motion_mode_t operating_mode, robot_geometry robot_geometry,
    pid_gains pid_gains, float max_motor_duty)
    : traction_control_gain_(1),
      lpf_alpha_(1),
      max_linear_acceleration_(std::numeric_limits<float>::max()),
      max_angular_acceleration_(std::numeric_limits<float>::max()) {
  operating_mode_ = operating_mode;
  robot_geometry_ = robot_geometry;
  pid_gains_ = pid_gains;
  max_motor_duty_ = max_motor_duty;
}

void SkidRobotMotionController::setAccelerationLimits(robot_velocities limits) {
  max_linear_acceleration_ = limits.linear_velocity;
  max_angular_acceleration_ = limits.angular_velocity;
}

robot_velocities SkidRobotMotionController::getAccelerationLimits() {
  robot_velocities returnstruct;
  returnstruct.angular_velocity = max_angular_acceleration_;
  returnstruct.linear_velocity = max_linear_acceleration_;
  return returnstruct;
}

void SkidRobotMotionController::setOperatingMode(
    robot_motion_mode_t operating_mode) {
  operating_mode_ = operating_mode;
}

robot_motion_mode_t SkidRobotMotionController::getOperatingMode() {
  return operating_mode_;
}

void SkidRobotMotionController::setRobotGeometry(
    robot_geometry robot_geometry) {
  robot_geometry_ = robot_geometry;
}

robot_geometry SkidRobotMotionController::getRobotGeometry() {
  return robot_geometry_;
}

void SkidRobotMotionController::setPidGains(pid_gains pid_gains) {
  pid_gains_ = pid_gains;
}

pid_gains SkidRobotMotionController::getPidGains() { return pid_gains_; }

void SkidRobotMotionController::setTractionGain(float traction_control_gain) {
  traction_control_gain_ = traction_control_gain;
}
float SkidRobotMotionController::getTractionGain() {
  return traction_control_gain_;
}

void SkidRobotMotionController::setMotorMaxDuty(float max_motor_duty) {
  max_motor_duty_ = max_motor_duty;
}
float SkidRobotMotionController::getMotorMaxDuty() { return max_motor_duty_; }

void SkidRobotMotionController::setFilterAlpha(float alpha) {
  lpf_alpha_ = alpha;
}
float SkidRobotMotionController::getFilterAlpha() { return lpf_alpha_; }

motor_data SkidRobotMotionController::runMotionControl(
    robot_velocities velocity_targets, motor_data current_duty_cycles,
    motor_data current_motor_speeds) {

    }
}  // namespace Control