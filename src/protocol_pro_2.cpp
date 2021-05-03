#include "protocol_pro_2.hpp"
namespace RoverRobotics {
Pro2ProtocolObject::Pro2ProtocolObject(
    const char *device, std::string new_comm_type,
    Control::robot_motion_mode_t robot_mode, Control::pid_gains pid,
    Control::angular_scaling_params angular_scale) {
  comm_type_ = new_comm_type;
  robot_mode_ = robot_mode;
  angular_scaling_params_ = angular_scale;
  robotstatus_ = {0};
  estop_ = false;
  motors_speeds_[FRONT_LEFT] = MOTOR_NEUTRAL_;
  motors_speeds_[FRONT_RIGHT] = MOTOR_NEUTRAL_;
  motors_speeds_[BACK_LEFT] = MOTOR_NEUTRAL_;
  motors_speeds_[BACK_RIGHT] = MOTOR_NEUTRAL_;
  pid_ = pid;

  get_params_file();

  skid_control_ = std::make_unique<Control::SkidRobotMotionController>(
      Control::TRACTION_CONTROL, robot_geometry_, pid_, MOTOR_MAX_, MOTOR_MIN_,
      left_trim_, right_trim_, geometric_decay_);
  skid_control_->setAngularScaling(angular_scaling_params_);
  skid_control_->setAccelerationLimits({5, 100000});
  skid_control_->setOpenLoopMaxRpm(600);
  skid_control_->setOperatingMode(robot_mode_);

  switch (robot_mode_) {
    case Control::OPEN_LOOP:
      robotmode_num_ = 0;
      skid_control_->setAccelerationLimits({std::numeric_limits<float>::max(),
                                            std::numeric_limits<float>::max()});
      break;
    case Control::TRACTION_CONTROL:
      robotmode_num_ = 1;
      break;
    case Control::INDEPENDENT_WHEEL:
      robotmode_num_ = 2;
      break;
  }

  vescArray_ = vesc::BridgedVescArray(
      std::vector<uint8_t>{VESC_IDS::FRONT_LEFT, VESC_IDS::FRONT_RIGHT,
                           VESC_IDS::BACK_LEFT, VESC_IDS::BACK_RIGHT});

  register_comm_base(device);
  write_to_robot_thread_ = std::thread([this]() { this->send_command(30); });

  // Create a motor update thread with 30 mili second sleep timer
  motor_speed_update_thread_ =
      std::thread([this]() { this->motors_control_loop(30); });
}

void Pro2ProtocolObject::get_params_file() {
  // open yaml
  std::ifstream robotconfig;
  std::string const HOME = std::getenv("HOME") ? std::getenv("HOME") : ".";
  robotconfig.open(HOME + "/rover.config");
  if (robotconfig.is_open()) {
    std::string line;
    std::cerr << "Loading config from " + HOME + "/rover.config" << std::endl;
    while (std::getline(robotconfig, line)) {
      std::vector<std::string> key =
          split(line, ":");  // get key-value pair into vector
      std::vector<std::string> values =
          split(key[1], " ");  // get all the values from this key
      if (key[0] == "trim") {  // check for key
        trimvalue_ = std::stod(values[0]);
        std::cerr << "new trim value " << trimvalue_ << std::endl;
      }
      // other params?
    }
  } else {
    std::cerr << "Failed to load config from " + HOME + "/rover.config"
              << std::endl;
    std::cerr << "Making a default config at " + HOME + "/rover.config"
              << std::endl;
    std::ofstream newrobotconfig;
    newrobotconfig.open(HOME + "/rover.config");
    newrobotconfig << "trim:0" << std::endl;
    newrobotconfig.close();
  }
}
void Pro2ProtocolObject::update_drivetrim(double delta) {
  // convert trim value to left and right motor power ratio
  if (-0.15 < trimvalue_ && trimvalue_ < .15) {
    trimvalue_ += delta;
    if (trimvalue_ >= 0) {
      left_trim_ = 1;
      right_trim_ = 1 - trimvalue_;
    } else if (trimvalue_ < 0) {
      right_trim_ = 1;
      left_trim_ = 1 + trimvalue_;
    }
    skid_control_->setTrim(left_trim_, right_trim_);
  }
  update_params(
      "trim",
      std::to_string(trimvalue_));  // update config file to have new trim value
}

void Pro2ProtocolObject::send_estop(bool estop) {
  robotstatus_mutex_.lock();
  estop_ = estop;
  robotstatus_mutex_.unlock();
}

robotData Pro2ProtocolObject::status_request() { return robotstatus_; }

robotData Pro2ProtocolObject::info_request() { return robotstatus_; }

void Pro2ProtocolObject::set_robot_velocity(double *control_array) {
  robotstatus_mutex_.lock();
  robotstatus_.cmd_linear_vel = control_array[0];
  robotstatus_.cmd_angular_vel = control_array[1];
  robotstatus_.cmd_ts = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch());
  robotstatus_mutex_.unlock();
}

void Pro2ProtocolObject::unpack_comm_response(std::vector<uint32_t> robotmsg) {
  if (auto parsedMsg = vescArray_.parseReceivedMessage(robotmsg)) {
    robotstatus_mutex_.lock();
    switch (parsedMsg->vescId) {
      case (FRONT_LEFT):
        robotstatus_.motor1_rpm = parsedMsg->rpm;
        robotstatus_.motor1_id = parsedMsg->vescId;
        robotstatus_.motor1_current = parsedMsg->current;
        break;
      case (FRONT_RIGHT):
        robotstatus_.motor2_rpm = parsedMsg->rpm;
        robotstatus_.motor2_id = parsedMsg->vescId;
        robotstatus_.motor2_current = parsedMsg->current;
        break;
      case (BACK_LEFT):
        robotstatus_.motor3_rpm = parsedMsg->rpm;
        robotstatus_.motor3_id = parsedMsg->vescId;
        robotstatus_.motor3_current = parsedMsg->current;
        break;
      case (BACK_RIGHT):
        robotstatus_.motor4_rpm = parsedMsg->rpm;
        robotstatus_.motor4_id = parsedMsg->vescId;
        robotstatus_.motor4_current = parsedMsg->current;
        break;
      default:
        break;
    }
    robotstatus_mutex_.unlock();
  }
}

bool Pro2ProtocolObject::is_connected() { return comm_base_->is_connected(); }

void Pro2ProtocolObject::register_comm_base(const char *device) {
  std::vector<uint32_t> setting;
  if (comm_type_ == "can") {
    try {
      comm_base_ = std::make_unique<CommCan>(
          device, [this](std::vector<uint32_t> c) { unpack_comm_response(c); },
          setting);
    } catch (int i) {
      throw(i);
    }
  } else
    throw(-2);
}

void Pro2ProtocolObject::send_command(int sleeptime) {
  while (true) {
    std::vector<std::vector<uint32_t>> messages;
    for (uint8_t vid = VESC_IDS::FRONT_LEFT; vid > VESC_IDS::BACK_RIGHT; vid++) {
      
      
      robotstatus_mutex_.lock();
      int32_t signedMotorCommand = static_cast<int32_t>(
          motors_speeds_[vid] * vesc::DUTY_COMMAND_SCALING_FACTOR);
      
      /* only use current control when robot is stopped to prevent wasted energy
       */
      bool useCurrentControl = motors_speeds_[vid] == 0 &&
                               robotstatus_.linear_vel == 0 &&
                               robotstatus_.angular_vel == 0;

      robotstatus_mutex_.unlock();

      comm_base_->write_to_device(
          vescArray_.buildCommandMessage((vesc::vescChannelCommand){
              .vescId = vid,
              .commandType = (useCurrentControl ? vesc::vescPacketFlags::CURRENT
                                                : vesc::vescPacketFlags::DUTY),
              .commandValue = (useCurrentControl ? 0 : signedMotorCommand)}));
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime));
  }
}

int Pro2ProtocolObject::cycle_robot_mode() {
  if (robotmode_num_ < 2) {
    robotmode_num_ += 1;
  } else
    robotmode_num_ = 0;
  switch (robotmode_num_) {
    case 0:
      skid_control_->setOperatingMode(Control::OPEN_LOOP);
      skid_control_->setAccelerationLimits({std::numeric_limits<float>::max(),
                                            std::numeric_limits<float>::max()});
      break;
    case 1:
      skid_control_->setOperatingMode(Control::TRACTION_CONTROL);
      skid_control_->setAccelerationLimits(
          {5, std::numeric_limits<float>::max()});
      break;
    case 2:
      skid_control_->setOperatingMode(Control::INDEPENDENT_WHEEL);
      skid_control_->setAccelerationLimits(
          {5, std::numeric_limits<float>::max()});
      break;
  }
  return robotmode_num_;
}

void Pro2ProtocolObject::motors_control_loop(int sleeptime) {
  float linear_vel, angular_vel, rpm_FL, rpm_FR, rpm_BL, rpm_BR;
  std::chrono::milliseconds time_last =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());
  std::chrono::milliseconds time_from_msg;

  while (true) {
    std::chrono::milliseconds time_now =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
    // get data from robot
    robotstatus_mutex_.lock();
    linear_vel = robotstatus_.cmd_linear_vel;
    angular_vel = robotstatus_.cmd_angular_vel;
    rpm_FL = robotstatus_.motor1_rpm;
    rpm_FR = robotstatus_.motor2_rpm;
    rpm_BL = robotstatus_.motor3_rpm;
    rpm_BR = robotstatus_.motor4_rpm;
    time_from_msg = robotstatus_.cmd_ts;
    robotstatus_mutex_.unlock();
    if (!estop_ &&
        (time_now - time_from_msg).count() <= CONTROL_LOOP_TIMEOUT_MS_) {
      auto duty_cycles = skid_control_->runMotionControl(
          {linear_vel, angular_vel}, {0, 0, 0, 0},
          {rpm_FL, rpm_FR, rpm_BL, rpm_BR});
      auto velocities = skid_control_->getMeasuredVelocities(
          {rpm_FL, rpm_FR, rpm_BL, rpm_BR});
      robotstatus_mutex_.lock();
      motors_speeds_[FRONT_LEFT] = duty_cycles.fl;
      motors_speeds_[FRONT_RIGHT] = duty_cycles.fr;
      motors_speeds_[BACK_LEFT] = duty_cycles.rl;
      motors_speeds_[BACK_RIGHT] = duty_cycles.rr;
      robotstatus_.linear_vel = velocities.linear_velocity;
      robotstatus_.angular_vel = velocities.angular_velocity;
      robotstatus_mutex_.unlock();
    } else {
      auto duty_cycles = skid_control_->runMotionControl(
          {0, 0}, {0, 0, 0, 0}, {rpm_FL, rpm_FR, rpm_BL, rpm_BR});
      auto velocities = skid_control_->getMeasuredVelocities(
          {rpm_FL, rpm_FR, rpm_BL, rpm_BR});
      robotstatus_mutex_.lock();
      motors_speeds_[FRONT_LEFT] = MOTOR_NEUTRAL_;
      motors_speeds_[FRONT_RIGHT] = MOTOR_NEUTRAL_;
      motors_speeds_[BACK_LEFT] = MOTOR_NEUTRAL_;
      motors_speeds_[BACK_RIGHT] = MOTOR_NEUTRAL_;
      robotstatus_.linear_vel = velocities.linear_velocity;
      robotstatus_.angular_vel = velocities.angular_velocity;
      robotstatus_mutex_.unlock();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime));
  }
}

void Pro2ProtocolObject::update_params(std::string replacing_key,
                                       std::string replacing_value) {
  // open previous config file;
  std::ifstream robotconfig;
  std::ofstream newrobotconfig;
  std::string const HOME = std::getenv("HOME") ? std::getenv("HOME") : ".";

  robotconfig.open(HOME + "/rover.config");
  newrobotconfig.open(HOME + "/rover.config.tmp");
  if (robotconfig.is_open()) {
    std::string line;
    std::cerr << "Updating config at " + HOME + "/rover.config" << std::endl;
    while (std::getline(robotconfig, line)) {
      std::vector<std::string> key =
          split(line, ":");  // get key-value pair into vector
      std::vector<std::string> values =
          split(key[1], " ");         // get all the values from this key
      if (key[0] == replacing_key) {  // check for key
        newrobotconfig << replacing_key << ":" << replacing_value
                       << std::endl;  // output modified value instead
      } else
        newrobotconfig << line;  // output original value
    }
  }
  robotconfig.close();
  newrobotconfig.close();
  std::string path = HOME + "/rover.config";
  int n = path.length();
  char robotconfig_path[n + 1];
  strcpy(robotconfig_path, path.c_str());
  path = HOME + "/rover.config.tmp";
  n = path.length();
  char newrobotconfig_path[n + 1];
  strcpy(newrobotconfig_path, path.c_str());
  if (rename(newrobotconfig_path, robotconfig_path) == 0)
    puts("Config File Updated");
  else
    perror("Error Saving config file");
}
std::vector<std::string> Pro2ProtocolObject::split(std::string str,
                                                   std::string token) {
  std::vector<std::string> result;
  while (str.size()) {
    int index = str.find(token);
    if (index != std::string::npos) {
      result.push_back(str.substr(0, index));
      str = str.substr(index + token.size());
      if (str.size() == 0) result.push_back(str);
    } else {
      result.push_back(str);
      str = "";
    }
  }
  return result;
}

}  // namespace RoverRobotics
