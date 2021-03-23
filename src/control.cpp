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

float PidController::runControl(float target, float measured) {
  /* current time */
  std::chrono::milliseconds time_now =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());

  /* delta time (mS) */
  float delta_time_ms = (time_now - time_last_).count();

  /* update time bookkeeping */
  time_last_ = time_now;

#ifdef DEBUG
  std::cerr << 'dt ' << delta_time_ms << std::endl;
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
  float d = (delta_time_ms / 1000) * kd_;

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

return output;

}

}  // namespace Control