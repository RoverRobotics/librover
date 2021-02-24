#include "protocol_pro.hpp"

namespace RoverRobotics {

ProProtocolObject::ProProtocolObject(const char *device,
                                     std::string new_comm_type,
                                     bool closed_loop, PidGains pid) {
  comm_type = new_comm_type;
  closed_loop_ = closed_loop;
  robotstatus_ = {0};
  estop_ = false;
  motors_speeds_[LEFT_MOTOR] = MOTOR_NEUTRAL;
  motors_speeds_[RIGHT_MOTOR] = MOTOR_NEUTRAL;
  motors_speeds_[FLIPPER_MOTOR] = MOTOR_NEUTRAL;
  std::vector<uint32_t> fast_data = {REG_MOTOR_FB_RPM_LEFT,
                                     REG_MOTOR_FB_RPM_RIGHT, EncoderInterval_0,
                                     EncoderInterval_1};
  std::vector<uint32_t> slow_data = {
      REG_MOTOR_FB_CURRENT_LEFT, REG_MOTOR_FB_CURRENT_RIGHT,
      REG_MOTOR_TEMP_LEFT,       REG_MOTOR_TEMP_RIGHT,
      REG_MOTOR_CHARGER_STATE,   BuildNO,
      BATTERY_VOLTAGE_A};
  pid_ = pid;
  motor1_control = OdomControl(closed_loop_, pid_, 1.5, 0);
  motor2_control = OdomControl(closed_loop_, pid_, 1.5, 0);

  register_comm_base(device);

  // Create a New Thread with 20 mili seconds sleep timer
  fast_data_write_thread =
      std::thread([this, fast_data]() { this->sendCommand(30, fast_data); });
  // Create a new Thread with 50 mili seconds sleep timer
  slow_data_write_thread =
      std::thread([this, slow_data]() { this->sendCommand(50, slow_data); });
  motor_commands_update_thread =
      std::thread([this]() { this->motors_update_loop(30); });
}

void ProProtocolObject::update_drivetrim(double value) { trimvalue = value; }

void ProProtocolObject::send_estop(bool estop) {
  robotstatus_mutex.lock();
  estop_ = estop;
  robotstatus_mutex.unlock();
}

robotData ProProtocolObject::status_request() { return robotstatus_; }

robotData ProProtocolObject::info_request() { return robotstatus_; }

void ProProtocolObject::set_robot_velocity(double *controlarray) {
  robotstatus_mutex.lock();
  robotstatus_.cmd_linear_vel = controlarray[0];
  robotstatus_.cmd_angular_vel = controlarray[1];
  motors_speeds_[FLIPPER_MOTOR] =
      (int)round(controlarray[2] + MOTOR_NEUTRAL) % MOTOR_MAX;
  robotstatus_.cmd_ts = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch());
  robotstatus_mutex.unlock();
}

void ProProtocolObject::motors_update_loop(int sleeptime) {
  double linear_vel;
  double angular_vel;
  double rpm1;
  double rpm2;

  std::chrono::milliseconds time_last =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());
  std::chrono::milliseconds time_from_msg;

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime));
    std::chrono::milliseconds time_now =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
    robotstatus_mutex.lock();
    int firmware = robotstatus_.robot_firmware;
    linear_vel = robotstatus_.cmd_linear_vel;
    angular_vel = robotstatus_.cmd_angular_vel;
    rpm1 = robotstatus_.motor1_rpm;
    rpm2 = robotstatus_.motor2_rpm;
    time_from_msg = robotstatus_.cmd_ts;
    robotstatus_mutex.unlock();
    float ctrl_update_elapsedtime = (time_now - time_from_msg).count();
    float pid_update_elapsedtime = (time_now - time_last).count();

    if (ctrl_update_elapsedtime > CONTROL_LOOP_TIMEOUT_MS || estop_) {
      robotstatus_mutex.lock();
      motors_speeds_[LEFT_MOTOR] = MOTOR_NEUTRAL;
      motors_speeds_[RIGHT_MOTOR] = MOTOR_NEUTRAL;
      motors_speeds_[FLIPPER_MOTOR] = MOTOR_NEUTRAL;
      motor1_control.reset();
      motor2_control.reset();
      robotstatus_mutex.unlock();
      time_last = time_now;
      continue;
    }

    if (angular_vel == 0) {
      if (linear_vel > 0) {
        angular_vel = trimvalue;
      } else if (linear_vel < 0) {
        angular_vel = -trimvalue;
      }
    }
    // !Applying some Skid-steer math
    double motor1_vel = linear_vel - 0.5 * angular_vel;
    double motor2_vel = linear_vel + 0.5 * angular_vel;
    if (motor1_vel == 0) motor1_control.reset();
    if (motor2_vel == 0) motor2_control.reset();

    double motor1_measured_vel = rpm1 / MOTOR_RPM_TO_MPS_RATIO;
    double motor2_measured_vel = rpm2 / MOTOR_RPM_TO_MPS_RATIO;
    robotstatus_mutex.lock();
    // motor speeds in m/s
    motors_speeds_[LEFT_MOTOR] =
        motor1_control.run(motor1_vel, motor1_measured_vel,
                           pid_update_elapsedtime / 1000, firmware);
    motors_speeds_[RIGHT_MOTOR] =
        motor2_control.run(motor2_vel, motor2_measured_vel,
                           pid_update_elapsedtime / 1000, firmware);

    // Convert to 8 bit Command
    motors_speeds_[LEFT_MOTOR] = motor1_control.boundMotorSpeed(
        int(round(motors_speeds_[LEFT_MOTOR] * 50 + MOTOR_NEUTRAL)), MOTOR_MAX,
        MOTOR_MIN);

    motors_speeds_[RIGHT_MOTOR] = motor2_control.boundMotorSpeed(
        int(round(motors_speeds_[RIGHT_MOTOR] * 50 + MOTOR_NEUTRAL)), MOTOR_MAX,
        MOTOR_MIN);
    robotstatus_mutex.unlock();
    time_last = time_now;
  }
}
void ProProtocolObject::unpack_comm_response(std::vector<uint32_t> robotmsg) {
  static std::vector<uint32_t> msgqueue;
  robotstatus_mutex.lock();
  msgqueue.insert(msgqueue.end(), robotmsg.begin(),
                  robotmsg.end());  // insert robotmsg to msg list
  // ! Delete bytes until valid start byte is found
  if ((unsigned char)msgqueue[0] != startbyte &&
      msgqueue.size() > RECEIVE_MSG_LEN) {
    int startbyte_index = 0;
    // !Did not find valid start byte in buffer
    while (msgqueue[startbyte_index] != startbyte &&
           startbyte_index < msgqueue.size())
      startbyte_index++;
    if (startbyte_index > msgqueue.size()) {
      msgqueue.clear();
      return;
    } else {
      // !Reconstruct the vector so that the start byte is at the 0 position
      std::vector<uint32_t> temp;
      for (int x = startbyte_index; x < msgqueue.size(); x++) {
        temp.push_back(msgqueue[x]);
      }
      msgqueue.clear();
      msgqueue.resize(0);
      msgqueue = temp;
      temp.clear();
    }
  }
  if ((unsigned char)msgqueue[0] == startbyte &&
      msgqueue.size() >= RECEIVE_MSG_LEN) {  // if valid start byte
    if (DEBUG) std::cerr << "start byte found!";
    unsigned char start_byte_read, data1, data2, dataNO, checksum,
        read_checksum;
    start_byte_read = (unsigned char)msgqueue[0];
    dataNO = (unsigned char)msgqueue[1];
    data1 = (unsigned char)msgqueue[2];
    data2 = (unsigned char)msgqueue[3];
    checksum = 255 - (dataNO + data1 + data2) % 255;
    read_checksum = (unsigned char)msgqueue[4];
    if (checksum == read_checksum) {  // verify checksum
      int b = (data1 << 8) + data2;
      switch (int(dataNO)) {
        case REG_PWR_TOTAL_CURRENT:
          break;
        case REG_MOTOR_FB_RPM_LEFT:
          robotstatus_.motor1_rpm = b;
          break;
        case REG_MOTOR_FB_RPM_RIGHT:  // motor2_rpm;
          robotstatus_.motor2_rpm = b;
          break;
        case REG_FLIPPER_FB_POSITION_POT1:
          robotstatus_.motor3_sensor1 = b;
          break;
        case REG_FLIPPER_FB_POSITION_POT2:
          robotstatus_.motor3_sensor2 = b;
          break;
        case REG_MOTOR_FB_CURRENT_LEFT:
          robotstatus_.motor1_current = b;
          break;
        case REG_MOTOR_FB_CURRENT_RIGHT:
          robotstatus_.motor2_current = b;
          break;
        case REG_MOTOR_ENCODER_COUNT_LEFT:

        case REG_MOTOR_ENCODER_COUNT_RIGHT:
        case REG_MOTOR_FAULT_FLAG_LEFT:
          robotstatus_.robot_fault_flag = b;
          break;
        case REG_MOTOR_TEMP_LEFT:
          robotstatus_.motor1_temp = b;
          break;
        case REG_MOTOR_TEMP_RIGHT:
          robotstatus_.motor2_temp = b;
          break;
        case REG_PWR_BAT_VOLTAGE_A:

        case REG_PWR_BAT_VOLTAGE_B:
        case EncoderInterval_0:
        case EncoderInterval_1:
        case EncoderInterval_2:
        case REG_ROBOT_REL_SOC_A:
        case REG_ROBOT_REL_SOC_B:
        case REG_MOTOR_CHARGER_STATE:
          robotstatus_.battery1_SOC = b;
          break;
        case BuildNO:
          robotstatus_.robot_firmware = b;
          break;
        case REG_PWR_A_CURRENT:
        case REG_PWR_B_CURRENT:
        case REG_MOTOR_FLIPPER_ANGLE:
          robotstatus_.motor3_angle = b;
          break;
        case to_computer_REG_MOTOR_SIDE_FAN_SPEED:
          robotstatus_.robot_fan_speed = b;
          break;
        case to_computer_REG_MOTOR_SLOW_SPEED:
        case BATTERY_STATUS_A:
        case BATTERY_STATUS_B:
        case BATTERY_MODE_A:
          robotstatus_.battery1_fault_flag = b;
          break;
        case BATTERY_MODE_B:
          robotstatus_.battery2_fault_flag = b;
          break;
        case BATTERY_TEMP_A:
          robotstatus_.battery1_temp = b;
          break;
        case BATTERY_TEMP_B:
          robotstatus_.battery2_temp = b;
          break;
        case BATTERY_VOLTAGE_A:
          robotstatus_.battery1_voltage = b;
          break;
        case BATTERY_VOLTAGE_B:
          robotstatus_.battery2_voltage = b;
          break;
        case BATTERY_CURRENT_A:
          robotstatus_.battery1_current = b;
          break;
        case BATTERY_CURRENT_B:
          robotstatus_.battery2_current = b;
          break;
      }
      // !Same battery system for both A and B on this robot
      robotstatus_.battery2_SOC = robotstatus_.battery1_SOC;
      // !THESE VALUES ARE NOT AVAILABLE ON ROVER PRO
      robotstatus_.motor1_id = 0;
      robotstatus_.motor1_mos_temp = 0;
      robotstatus_.motor2_id = 0;
      robotstatus_.motor2_mos_temp = 0;
      robotstatus_.motor3_id = 0;
      robotstatus_.motor3_rpm = 0;
      robotstatus_.motor3_current = 0;
      robotstatus_.motor3_temp = 0;
      robotstatus_.motor3_mos_temp = 0;
      robotstatus_.motor4_id = 0;
      robotstatus_.motor4_rpm = 0;
      robotstatus_.motor4_current = 0;
      robotstatus_.motor4_temp = 0;
      robotstatus_.motor4_mos_temp = 0;
      robotstatus_.robot_guid = 0;
      robotstatus_.robot_speed_limit = 0;

      robotstatus_.linear_vel =
          0.5 * (robotstatus_.motor1_rpm / MOTOR_RPM_TO_MPS_RATIO +
                 robotstatus_.motor2_rpm / MOTOR_RPM_TO_MPS_RATIO);

      robotstatus_.angular_vel =
          ((robotstatus_.motor1_rpm / MOTOR_RPM_TO_MPS_RATIO) -
           (robotstatus_.motor2_rpm / MOTOR_RPM_TO_MPS_RATIO)) *
          odom_angular_coef_ * odom_traction_factor_;

      std::vector<uint32_t> temp;
      // !Remove processed msg from queue
      for (int x = RECEIVE_MSG_LEN; x < msgqueue.size(); x++) {
        temp.push_back(msgqueue[x]);
      }
      msgqueue.clear();
      msgqueue.resize(0);
      msgqueue = temp;
      temp.clear();
    } else {  // !Found start byte but the msg contents were invalid, throw away
              // broken message
      if (DEBUG) std::cerr << "failed checksum" << std::endl;
      std::vector<uint32_t> temp;
      for (int x = 1; x < msgqueue.size(); x++) {
        temp.push_back(msgqueue[x]);
      }
      msgqueue.clear();
      msgqueue.resize(0);
      msgqueue = temp;
      temp.clear();
    }

  } else {
    // !ran out of data; waiting for more
    if (DEBUG) std::cerr << "no start byte found!";
  }
  if (DEBUG) std::cerr << std::endl;
  robotstatus_mutex.unlock();
}

bool ProProtocolObject::is_connected() { comm_base->is_connected(); }

void ProProtocolObject::register_comm_base(const char *device) {
  if (comm_type == "serial") {
    std::cerr << "making serial connection" << std::endl;
    std::vector<uint32_t> setting_;
    setting_.push_back(termios_baud_code);
    setting_.push_back(RECEIVE_MSG_LEN);
    comm_base = std::make_unique<CommSerial>(
        device, [this](std::vector<uint32_t> c) { unpack_comm_response(c); },
        setting_);
  } else {  // generic case
    std::cerr << "unknown or not supported communication type " << comm_type
              << std::endl;
    throw(-1);
  }
}

void ProProtocolObject::sendCommand(int sleeptime,
                                    std::vector<uint32_t> datalist) {
  while (true) {
    for (int x : datalist) {
      if (comm_type == "serial") {
        robotstatus_mutex.lock();
        std::vector<uint32_t> write_buffer = {
            (unsigned char)startbyte,
            (unsigned char)int(motors_speeds_[LEFT_MOTOR]),
            (unsigned char)int(motors_speeds_[RIGHT_MOTOR]),
            (unsigned char)int(motors_speeds_[FLIPPER_MOTOR]),
            (unsigned char)requestbyte,
            (unsigned char)x};

        write_buffer.push_back(
            (char)255 - ((unsigned char)int(motors_speeds_[LEFT_MOTOR]) +
                         (unsigned char)int(motors_speeds_[RIGHT_MOTOR]) +
                         (unsigned char)int(motors_speeds_[FLIPPER_MOTOR]) +
                         requestbyte + x) %
                            255);
        comm_base->write_to_device(write_buffer);
        robotstatus_mutex.unlock();
      } else if (comm_type == "can") {
        return;  //* no CAN for rover pro
      } else {   //! How did you get here?
        return;  // TODO: Return error ?
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime));
    }
  }
}

}  // namespace RoverRobotics
