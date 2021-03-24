#include "control.hpp"

#include <math.h>

#include <limits>

namespace Control {
/* functions */

motor_data computeSkidSteerWheelSpeeds(robot_velocities target_velocities,
                                       robot_geometry robot_geometry) {
  /* see documentation for math */
  /* radius of robot's stance */
  float rs = sqrt(pow(0.5 * robot_geometry.wheel_base, 2) +
                  pow(0.5 * robot_geometry.intra_axle_distance, 2));

  /* circumference of robot's stance (meters) */
  float cs = 2 * M_PI * rs;

  /* travel rate(m/s) */
  float left_travel_rate = target_velocities.linear_velocity -
                           0.5 * target_velocities.angular_velocity;
  float right_travel_rate = target_velocities.linear_velocity +
                            0.5 * target_velocities.angular_velocity;

  /* convert (m/s) -> rpm */
  float left_wheel_speed =
      (left_travel_rate / robot_geometry.wheel_radius) / RPM_TO_RADS_SEC;
  float right_wheel_speed =
      (right_travel_rate / robot_geometry.wheel_radius) / RPM_TO_RADS_SEC;

  motor_data returnstruct = {left_wheel_speed, right_wheel_speed,
                             left_wheel_speed, right_wheel_speed};
  return returnstruct;
}
robot_velocities computeVelocitiesFromWheelspeeds(
    motor_data wheel_speeds, robot_geometry robot_geometry) {
  /* see documentation for math */
  /* radius of robot's stance */
  float rs = sqrt(pow(0.5 * robot_geometry.wheel_base, 2) +
                  pow(0.5 * robot_geometry.intra_axle_distance, 2));

  /* circumference of robot's stance (meters) */
  float cs = 2 * M_PI * rs;

  /* translate wheelspeed (rpm) into travel rate(m/s) */
  float left_travel_rate = std::min(wheel_speeds.fl, wheel_speeds.rl) *
                           RPM_TO_RADS_SEC * robot_geometry.wheel_radius;
  float right_travel_rate = std::min(wheel_speeds.fr, wheel_speeds.rr) *
                            RPM_TO_RADS_SEC * robot_geometry.wheel_radius;

  /* difference between left and right travel rates */
  float travel_differential = right_travel_rate - left_travel_rate;

  /* compute velocities */
  float linear_velocity = std::min(left_travel_rate, right_travel_rate);
  float angular_velocity =
      travel_differential / cs;  // possibly add traction factor here

  robot_velocities returnstruct;
  returnstruct.linear_velocity = linear_velocity;
  returnstruct.angular_velocity = angular_velocity;
  return returnstruct;
}

robot_velocities limitAcceleration(robot_velocities target_velocities,
                                   robot_velocities measured_velocities,
                                   robot_velocities delta_v_limits, float dt) {
  /* compute proposed acceleration */
  float linear_acceleration = (target_velocities.linear_velocity -
                               measured_velocities.linear_velocity) /
                              dt;
  float angular_acceleration = (target_velocities.angular_velocity -
                                measured_velocities.angular_velocity) /
                               dt;

  /* clip the proposed acceleration into an acceptable acceleration */
  if (linear_acceleration > delta_v_limits.linear_velocity) {
    linear_acceleration = delta_v_limits.linear_velocity;
  }
  if (linear_acceleration < -delta_v_limits.linear_velocity) {
    linear_acceleration = -delta_v_limits.linear_velocity;
  }
  if (angular_acceleration > delta_v_limits.angular_velocity) {
    angular_acceleration = delta_v_limits.angular_velocity;
  }
  if (angular_acceleration < -delta_v_limits.angular_velocity) {
    angular_acceleration = -delta_v_limits.angular_velocity;
  }

  /* calculate new velocities */
  robot_velocities return_velocities;
  return_velocities.linear_velocity =
      measured_velocities.linear_velocity + linear_acceleration * dt;
  return_velocities.angular_velocity =
      measured_velocities.angular_velocity + angular_acceleration * dt;
  return return_velocities;
}

/* classes */
PidController::PidController(struct pid_gains pid_gains, std::string name)
    : /* defaults */
      integral_error_(0),
      integral_error_limit_(std::numeric_limits<float>::max()),
      pos_max_output_(std::numeric_limits<float>::max()),
      neg_max_output_(std::numeric_limits<float>::lowest()),
      time_last_(std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())) {
  name_ = name;
  kp_ = pid_gains.kp;
  kd_ = pid_gains.kd;
  ki_ = pid_gains.ki;
};

PidController::PidController(struct pid_gains pid_gains,
                             pid_output_limits pid_output_limits,
                             std::string name)
    : /* defaults */
      integral_error_(0),
      integral_error_limit_(std::numeric_limits<float>::max()),
      time_last_(std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())) {
  name_ = name;
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

void PidController::writePidDataToCsv(std::ofstream log_file,
                                      pid_outputs data) {
  log_file << "pid," << data.name << "," << data.time << "," << data.dt << ","
           << data.pid_output << "," << data.error << "," << data.integral_error
           << "," << data.target_value << "," << data.measured_value << ","
           << data.kp << "," << data.ki << "," << data.kd << "," << std::endl;
}

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
  returnstruct.name = name_;
  returnstruct.dt = delta_time;
  returnstruct.time = time_now.count();
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
    : log_folder_path_("~/Documents/rover/logs/"),
      operating_mode_(OPEN_LOOP),
      traction_control_gain_(1),
      max_motor_duty_(100),
      time_last_(std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())) {
#ifdef DEBUG
  /*open a log file to store control data*/
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  std::ostringstream oss;
  oss << std::put_time(&tm, "%d-%m-%Y-%H-%M-%S");
  auto filename = oss.str();

  log_file_.open(log_folder_path_ + filename + ".csv")
#endif
}

SkidRobotMotionController::SkidRobotMotionController(
    robot_motion_mode_t operating_mode, robot_geometry robot_geometry,
    pid_gains pid_gains, float max_motor_duty)
    : log_folder_path_("~/Documents/rover/logs/"),
      traction_control_gain_(1),
      lpf_alpha_(1),
      max_linear_acceleration_(std::numeric_limits<float>::max()),
      max_angular_acceleration_(std::numeric_limits<float>::max()),
      time_last_(std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())) {
  operating_mode_ = operating_mode;
  robot_geometry_ = robot_geometry;
  pid_gains_ = pid_gains;
  max_motor_duty_ = max_motor_duty;
  switch (operating_mode) {
    case OPEN_LOOP:
      break;
    case INDEPENDENT_WHEEL:
      /* one pid per wheel */
      pid_controller_fl_ =
          std::make_unique<PidController>(pid_gains_, "pid_front_left");
      pid_controller_fr_ =
          std::make_unique<PidController>(pid_gains_, "pid_front_right");
      pid_controller_rl_ =
          std::make_unique<PidController>(pid_gains_, "pid_rear_left");
      pid_controller_rr_ =
          std::make_unique<PidController>(pid_gains_, "pid_rear_right");
      break;
    case TRACTION_CONTROL:
      /* one pid per side */
      pid_controller_left_ =
          std::make_unique<PidController>(pid_gains_, "pid_left");
      pid_controller_right_ =
          std::make_unique<PidController>(pid_gains_, "pid_right");
      break;
    default:
      /* probably throw exception here */
      break;
  }
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

motor_data SkidRobotMotionController::computeMotorCommandsTc_(
    motor_data target_wheel_speeds, motor_data current_motor_speeds) {
  /* run pid, 1 per side */
  pid_outputs l_pid_output = pid_controller_left_->runControl(
      target_wheel_speeds.fl,
      std::min(current_motor_speeds.fl, current_motor_speeds.rl));

  pid_outputs r_pid_output = pid_controller_right_->runControl(
      target_wheel_speeds.fr,
      std::min(current_motor_speeds.fr, current_motor_speeds.rr));

#ifdef DEBUG
  PidController::writePidDataToCsv(log_file_, l_pid_output);
  PidController::writePidDataToCsv(log_file_, r_pid_output);
#endif

  /* math to split the torque distribution */
  motor_data power_proposals = {
      l_pid_output.pid_output, r_pid_output.pid_output, l_pid_output.pid_output,
      r_pid_output.pid_output};

  /* right side */
  if (current_motor_speeds.fr >= current_motor_speeds.rr) {
    /* scale down FRONT RIGHT power */
    power_proposals.fr *= current_motor_speeds.rr / current_motor_speeds.fr;
  } else {
    /* scale down REAR RIGHT power */
    power_proposals.rr *= current_motor_speeds.fr / current_motor_speeds.rr;
  }
  /* left side */
  if (current_motor_speeds.fl >= current_motor_speeds.rl) {
    /* scale down FRONT LEFT power */
    power_proposals.fl *= current_motor_speeds.rl / current_motor_speeds.fl;
  } else {
    /* scale down REAR LEFT power */
    power_proposals.rl *= current_motor_speeds.fl / current_motor_speeds.rl;
  }

  return power_proposals;
}

motor_data SkidRobotMotionController::clipDutyCycles_(
    motor_data proposed_duties) {
  proposed_duties.fr =
      std::clamp(proposed_duties.fr, -max_motor_duty_, max_motor_duty_);
  proposed_duties.fl =
      std::clamp(proposed_duties.fl, -max_motor_duty_, max_motor_duty_);
  proposed_duties.rr =
      std::clamp(proposed_duties.rr, -max_motor_duty_, max_motor_duty_);
  proposed_duties.rl =
      std::clamp(proposed_duties.rl, -max_motor_duty_, max_motor_duty_);
  return proposed_duties;
}

motor_data SkidRobotMotionController::runMotionControl(
    robot_velocities velocity_targets, motor_data current_duty_cycles,
    motor_data current_motor_speeds) {
  /* take the time*/
  std::chrono::milliseconds time_now =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());
  float delta_time = (time_now - time_last_).count() / 1000;

  /* get estimated robot velocities */
  robot_velocities measured_velocities =
      computeVelocitiesFromWheelspeeds(current_motor_speeds, robot_geometry_);

#ifdef DEBUG
  std::cerr << 'est robot lin vel:  ' << robot_velocities.linear_velocity
            << std::endl;
  std::cerr << 'est robot ang vel:  ' << robot_velocities.angular_velocity
            << std::endl;
#endif

  /* limit acceleration */
  robot_velocities velocity_commands;
  robot_velocities acceleration_limits = {max_linear_acceleration_,
                                          max_angular_acceleration_};
  velocity_commands = limitAcceleration(velocity_targets, measured_velocities,
                                        acceleration_limits, delta_time);

  /* get target wheelspeeds from velocities */
  motor_data target_wheel_speeds =
      computeSkidSteerWheelSpeeds(velocity_commands, robot_geometry_);

  /* do control */
  motor_data motor_duties;
  switch (operating_mode_) {
    case OPEN_LOOP:
      break;
    case INDEPENDENT_WHEEL:
      break;
    case TRACTION_CONTROL:
      return clipDutyCycles_(
          computeMotorCommandsTc_(target_wheel_speeds, current_motor_speeds));

    default:
      return {0, 0, 0, 0};
  }
}
}  // namespace Control