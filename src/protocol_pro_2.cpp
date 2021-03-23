#include "protocol_pro_2.hpp"

namespace RoverRobotics {
Pro2ProtocolObject::Pro2ProtocolObject(const char *device,
                                       std::string new_comm_type,
                                       bool closed_loop, PidGains pid) {
  comm_type_ = new_comm_type;
  closed_loop_ = closed_loop;
  robotstatus_ = {0};
  estop_ = false;
  motors_speeds_[FRONT_LEFT_MOTOR] = MOTOR_NEUTRAL_;
  motors_speeds_[FRONT_RIGHT_MOTOR] = MOTOR_NEUTRAL_;
  motors_speeds_[BACK_LEFT_MOTOR] = MOTOR_NEUTRAL_;
  motors_speeds_[BACK_RIGHT_MOTOR] = MOTOR_NEUTRAL_;
  motor1_control_ = OdomControl(closed_loop_, pid_, 5, -5);
  motor2_control_ = OdomControl(closed_loop_, pid_, 5, -5);
  motor3_control_ = OdomControl(closed_loop_, pid_, 5, -5);
  motor4_control_ = OdomControl(closed_loop_, pid_, 5, -5);

  pid_ = pid;
  register_comm_base(device);
  // TODO
  // std::vector<uint32_t> fast_data = {REG_MOTOR_FB_RPM_LEFT,
  //                                    REG_MOTOR_FB_RPM_RIGHT,
  //                                    EncoderInterval_0, EncoderInterval_1};
  std::vector<uint32_t> slow_data = {
      REG_MOTOR_FB_CURRENT_LEFT, REG_MOTOR_FB_CURRENT_RIGHT,
      REG_MOTOR_TEMP_LEFT,       REG_MOTOR_TEMP_RIGHT,
      REG_MOTOR_CHARGER_STATE,   BuildNO,
      BATTERY_VOLTAGE_A};
  // Create a New Thread with 30 mili seconds sleep timer
  // fast_data_write_thread_ =
  //     std::thread([this, fast_data]() { this->sendCommand(30, fast_data); });
  // // Create a new Thread with 50 mili seconds sleep timer
  slow_data_write_thread_ =
      std::thread([this, slow_data]() { this->sendCommand(30, slow_data); });
  // Create a motor update thread with 30 mili second sleep timer
  motor_commands_update_thread_ =
      std::thread([this]() { this->motors_control_loop(30); });
}

void Pro2ProtocolObject::update_drivetrim(double value) { trimvalue_ += value; }

void Pro2ProtocolObject::send_estop(bool estop) {
  robotstatus_mutex_.lock();
  estop_ = estop;
  robotstatus_mutex_.unlock();
}

robotData Pro2ProtocolObject::status_request() { return robotstatus_; }

robotData Pro2ProtocolObject::info_request() { return robotstatus_; }

void Pro2ProtocolObject::set_robot_velocity(double *controlarray) {
  robotstatus_mutex_.lock();
  robotstatus_.cmd_linear_vel = controlarray[0];
  robotstatus_.cmd_angular_vel = controlarray[1];
  robotstatus_.cmd_ts = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch());
  robotstatus_mutex_.unlock();
}
// REMOVE?
// void Pro2ProtocolObject::send_speed(double *controlarray) {
//     // prevent constant lock
//     writemutex.lock();
//     std::chrono::steady_clock::time_point motor1_prev_temp;
//     std::chrono::steady_clock::time_point motor2_prev_temp;
//     std::chrono::steady_clock::time_point motor3_prev_temp;
//     std::chrono::steady_clock::time_point motor4_prev_temp;
//     int firmware = robotstatus_.robot_firmware;
//     double rpm1 = robotstatus_.motor1_rpm;
//     double rpm2 = robotstatus_.motor2_rpm;
//     double rpm3 = robotstatus_.motor3_rpm;
//     double rpm4 = robotstatus_.motor4_rpm;

//     writemutex.unlock();

//     writemutex.lock();
//     if (!estop_) {
//         double linear_rate = controlarray[0];
//         double turn_rate = controlarray[1];
//         double flipper_rate = controlarray[2];
//         std::cerr << linear_rate << " " << turn_rate << " " << flipper_rate;
//         // apply trim value

//         if (turn_rate == 0) {
//             if (linear_rate > 0) {
//                 turn_rate = trimvalue;
//             } else if (linear_rate < 0) {
//                 turn_rate = -trimvalue;
//             }
//         }
//         double diff_vel_commanded = turn_rate;
//         double motor1_vel = linear_rate - 0.5 * diff_vel_commanded;
//         double motor2_vel = linear_rate + 0.5 * diff_vel_commanded;
//         double motor3_vel = linear_rate - 0.5 * diff_vel_commanded;
//         double motor4_vel = linear_rate + 0.5 * diff_vel_commanded;
//         double motor1_measured_vel =
//             rpm1 / MOTOR_RPM_TO_MPS_RATIO + MOTOR_RPM_TO_MPS_CFB;
//         double motor2_measured_vel =
//             rpm2 / MOTOR_RPM_TO_MPS_RATIO + MOTOR_RPM_TO_MPS_CFB;
//         double motor3_measured_vel =
//             rpm3 / MOTOR_RPM_TO_MPS_RATIO + MOTOR_RPM_TO_MPS_CFB;
//         double motor4_measured_vel =
//             rpm4 / MOTOR_RPM_TO_MPS_RATIO + MOTOR_RPM_TO_MPS_CFB;
//         std::cerr << "commanded motor speed from ROS (m/s): "
//                   << "left:" << motor1_vel << " right:" << motor2_vel <<
//                   std::endl;
//         std::cerr << "measured motor speed (m/s)"
//                   << " left:" << motor1_measured_vel
//                   << " right:" << motor2_measured_vel << std::endl;
//         std::chrono::steady_clock::time_point current_time =
//             std::chrono::steady_clock::now();
//         motors_speeds_[0] = motor1_control.run(
//             motor1_vel, motor1_measured_vel,
//             std::chrono::duration_cast<std::chrono::microseconds>(current_time
//             -
//                                                                   motor1_prev_temp)
//                     .count() /
//                 1000000.0,
//             firmware);
//         motors_speeds_[1] = motor2_control.run(
//             motor2_vel, motor2_measured_vel,
//             std::chrono::duration_cast<std::chrono::microseconds>(current_time
//             -
//                                                                   motor2_prev_temp)
//                     .count() /
//                 1000000.0,
//             firmware);
//         motors_speeds_[2] = motor3_control.run(
//             motor3_vel, motor3_measured_vel,
//             std::chrono::duration_cast<std::chrono::microseconds>(current_time
//             -
//                                                                   motor3_prev_temp)
//                     .count() /
//                 1000000.0,
//             firmware);
//         motors_speeds_[3] = motor4_control.run(
//             motor4_vel, motor4_measured_vel,
//             std::chrono::duration_cast<std::chrono::microseconds>(current_time
//             -
//                                                                   motor4_prev_temp)
//                     .count() /
//                 1000000.0,
//             firmware);

//         std::cerr << "open loop motor command"
//                   << " left:" << (int)round(motor1_vel)
//                   << " right:" << (int)round(motor2_vel) << std::endl;
//         std::cerr << "closed loop motor command"
//                   << " left:" << motors_speeds_[0] << " right:" <<
//                   motors_speeds_[1]
//                   << std::endl;
//     } else {
//         motors_speeds_[0] = MOTOR_NEUTRAL;
//         motors_speeds_[1] = MOTOR_NEUTRAL;
//         motors_speeds_[2] = MOTOR_NEUTRAL;
//         motors_speeds_[3] = MOTOR_NEUTRAL;
//     }

//     writemutex.unlock();
// }

void Pro2ProtocolObject::unpack_comm_response(std::vector<uint32_t> robotmsg) {
  static std::vector<uint32_t> msgqueue;
  robotstatus_mutex_.lock();
  for (int i = 0; i < 8; i++){
    std::cerr << robotmsg[i];
  }
  //   msgqueue.insert(msgqueue.end(), robotmsg.begin(),
  //                   robotmsg.end());  // insert robotmsg to msg list
  //   // ! Delete bytes until valid start byte is found

  //   if ((unsigned char)msgqueue[0] != startbyte_ &&
  //       msgqueue.size() > RECEIVE_MSG_LEN_) {
  //     int startbyte_index = 0;
  //     // !Did not find valid start byte in buffer
  //     while (msgqueue[startbyte_index] != startbyte_ &&
  //            startbyte_index < msgqueue.size())
  //       startbyte_index++;
  //     if (startbyte_index > msgqueue.size()) {
  //       msgqueue.clear();
  //       return;
  //     } else {
  //       // !Reconstruct the vector so that the start byte is at the 0
  //       position std::vector<uint32_t> temp; for (int x = startbyte_index; x
  //       < msgqueue.size(); x++) {
  //         temp.push_back(msgqueue[x]);
  //       }
  //       msgqueue.clear();
  //       msgqueue.resize(0);
  //       msgqueue = temp;
  //       temp.clear();
  //     }
  //   }
  //   if ((unsigned char)msgqueue[0] == startbyte_ &&
  //       msgqueue.size() >= RECEIVE_MSG_LEN_) {  // if valid start byte
  //     unsigned char start_byte_read, data1, data2, dataNO, checksum,
  //         read_checksum;
  //     start_byte_read = (unsigned char)msgqueue[0];
  //     dataNO = (unsigned char)msgqueue[1];
  //     data1 = (unsigned char)msgqueue[2];
  //     data2 = (unsigned char)msgqueue[3];
  //     checksum = 255 - (dataNO + data1 + data2) % 255;
  //     read_checksum = (unsigned char)msgqueue[4];
  //     if (checksum == read_checksum) {  // verify checksum
  //       int b = (data1 << 8) + data2;
  //       switch (int(dataNO)) {
  //         case REG_PWR_TOTAL_CURRENT:
  //           break;
  //         case REG_MOTOR_FB_RPM_LEFT:
  //           robotstatus_.motor1_rpm = b;
  //           break;
  //         case REG_MOTOR_FB_RPM_RIGHT:  // motor2_rpm;
  //           robotstatus_.motor2_rpm = b;
  //           break;
  //         case REG_FLIPPER_FB_POSITION_POT1:
  //           robotstatus_.motor3_sensor1 = b;
  //           break;
  //         case REG_FLIPPER_FB_POSITION_POT2:
  //           robotstatus_.motor3_sensor2 = b;
  //           break;
  //         case REG_MOTOR_FB_CURRENT_LEFT:
  //           robotstatus_.motor1_current = b;
  //           break;
  //         case REG_MOTOR_FB_CURRENT_RIGHT:
  //           robotstatus_.motor2_current = b;
  //           break;
  //         case REG_MOTOR_ENCODER_COUNT_LEFT:

  //         case REG_MOTOR_ENCODER_COUNT_RIGHT:
  //         case REG_MOTOR_FAULT_FLAG_LEFT:
  //           robotstatus_.robot_fault_flag = b;
  //           break;
  //         case REG_MOTOR_TEMP_LEFT:
  //           robotstatus_.motor1_temp = b;
  //           break;
  //         case REG_MOTOR_TEMP_RIGHT:
  //           robotstatus_.motor2_temp = b;
  //           break;
  //         case REG_PWR_BAT_VOLTAGE_A:

  //         case REG_PWR_BAT_VOLTAGE_B:
  //         case EncoderInterval_0:
  //         case EncoderInterval_1:
  //         case EncoderInterval_2:
  //         case REG_ROBOT_REL_SOC_A:
  //         case REG_ROBOT_REL_SOC_B:
  //         case REG_MOTOR_CHARGER_STATE:
  //           robotstatus_.battery1_SOC = b;
  //           break;
  //         case BuildNO:
  //           robotstatus_.robot_firmware = b;
  //           break;
  //         case REG_PWR_A_CURRENT:
  //         case REG_PWR_B_CURRENT:
  //         case REG_MOTOR_FLIPPER_ANGLE:
  //           robotstatus_.motor3_angle = b;
  //           break;
  //         case to_computer_REG_MOTOR_SIDE_FAN_SPEED:
  //           robotstatus_.robot_fan_speed = b;
  //           break;
  //         case to_computer_REG_MOTOR_SLOW_SPEED:
  //         case BATTERY_STATUS_A:
  //         case BATTERY_STATUS_B:
  //         case BATTERY_MODE_A:
  //           robotstatus_.battery1_fault_flag = b;
  //           break;
  //         case BATTERY_MODE_B:
  //           robotstatus_.battery2_fault_flag = b;
  //           break;
  //         case BATTERY_TEMP_A:
  //           robotstatus_.battery1_temp = b;
  //           break;
  //         case BATTERY_TEMP_B:
  //           robotstatus_.battery2_temp = b;
  //           break;
  //         case BATTERY_VOLTAGE_A:
  //           robotstatus_.battery1_voltage = b;
  //           break;
  //         case BATTERY_VOLTAGE_B:
  //           robotstatus_.battery2_voltage = b;
  //           break;
  //         case BATTERY_CURRENT_A:
  //           robotstatus_.battery1_current = b;
  //           break;
  //         case BATTERY_CURRENT_B:
  //           robotstatus_.battery2_current = b;
  //           break;
  //       }
  //       // !Same battery system for both A and B on this robot
  //       robotstatus_.battery2_SOC = robotstatus_.battery1_SOC;
  //       // !THESE VALUES ARE NOT AVAILABLE ON ROVER PRO
  //       robotstatus_.motor1_id = 0;
  //       robotstatus_.motor1_mos_temp = 0;
  //       robotstatus_.motor2_id = 0;
  //       robotstatus_.motor2_mos_temp = 0;
  //       robotstatus_.motor3_id = 0;
  //       robotstatus_.motor3_rpm = 0;
  //       robotstatus_.motor3_current = 0;
  //       robotstatus_.motor3_temp = 0;
  //       robotstatus_.motor3_mos_temp = 0;
  //       robotstatus_.motor4_id = 0;
  //       robotstatus_.motor4_rpm = 0;
  //       robotstatus_.motor4_current = 0;
  //       robotstatus_.motor4_temp = 0;
  //       robotstatus_.motor4_mos_temp = 0;
  //       robotstatus_.robot_guid = 0;
  //       robotstatus_.robot_speed_limit = 0;

  //       robotstatus_.linear_vel =
  //           0.5 * (robotstatus_.motor1_rpm / MOTOR_RPM_TO_MPS_RATIO_ +
  //                  robotstatus_.motor2_rpm / MOTOR_RPM_TO_MPS_RATIO_);

  //       robotstatus_.angular_vel =
  //           ((robotstatus_.motor1_rpm / MOTOR_RPM_TO_MPS_RATIO_) -
  //            (robotstatus_.motor2_rpm / MOTOR_RPM_TO_MPS_RATIO_)) *
  //           odom_angular_coef_ * odom_traction_factor_;

  //       std::vector<uint32_t> temp;
  //       // !Remove processed msg from queue
  //       for (int x = RECEIVE_MSG_LEN_; x < msgqueue.size(); x++) {
  //         temp.push_back(msgqueue[x]);
  //       }
  //       msgqueue.clear();
  //       msgqueue.resize(0);
  //       msgqueue = temp;
  //       temp.clear();
  //     } else {  // !Found start byte but the msg contents were invalid, throw
  //     away
  //               // broken message
  //       std::vector<uint32_t> temp;
  //       for (int x = 1; x < msgqueue.size(); x++) {
  //         temp.push_back(msgqueue[x]);
  //       }
  //       msgqueue.clear();
  //       msgqueue.resize(0);
  //       msgqueue = temp;
  //       temp.clear();
  //     }

  //   } else {
  //     // !ran out of data; waiting for more
  //   }
  robotstatus_mutex_.unlock();
}

bool Pro2ProtocolObject::is_connected() { return comm_base_->is_connected(); }

void Pro2ProtocolObject::register_comm_base(const char *device) {
  std::vector<uint32_t> setting;
  setting.push_back(termios_baud_code_);
  setting.push_back(RECEIVE_MSG_LEN_);
  if (comm_type_ == "serial") {
    std::cerr << "making serial connection" << std::endl;
    try {
      comm_base_ = std::make_unique<CommSerial>(
          device, [this](std::vector<uint32_t> c) { unpack_comm_response(c); },
          setting);
    } catch (int i) {
      throw(i);
    }
  } else if (comm_type_ == "can") {
    try {
      comm_base_ = std::make_unique<CommCan>(
          device, [this](std::vector<uint32_t> c) { unpack_comm_response(c); },
          setting);
    } catch (int i) {
      throw(i);
    }
  }
}

void Pro2ProtocolObject::sendCommand(int sleeptime,
                                     std::vector<uint32_t> datalist) {
  while (true) {
    for (int x : datalist) {
      if (comm_type_ == "serial") {
        // TODO

        // writemutex.lock();
        // write_buffer[0] = (unsigned char)253;
        // write_buffer[1] = (unsigned char)motors_speeds_[0];  // left motor
        // write_buffer[2] = (unsigned char)motors_speeds_[1];  // right motor
        // write_buffer[3] = (unsigned char)motors_speeds_[2];  // flipper
        // write_buffer[4] = (unsigned char)10;
        // write_buffer[5] = (unsigned char)x;  // Param 2:
        // // Calculate Checksum
        // write_buffer[6] =
        //     (char)255 - (write_buffer[1] + write_buffer[2] + write_buffer[3]
        //     +
        //                  write_buffer[4] + write_buffer[5]) %
        //                     255;
        // comm_base->writetodevice(write_buffer);
        // // std::cerr << "To Robot: ";
        // // for (int i = 0; i < sizeof(write_buffer); i++) {
        // //   std::cerr << int(write_buffer[i]) << " ";
        // //   // std::cerr <<  std::dec <<int(write_buffer[i]) << " ";
        // // }
        // // std::cout << std::endl;
        // writemutex.unlock();
      } else if (comm_type_ == "can") {
        robotstatus_mutex_.lock();
        for (int i = 0; i < 4; i++) {
          int32_t v = static_cast<int32_t>(motors_speeds_[i] * 100000.0);
          std::vector<uint32_t> write_buffer = {
              i | 0x80000000U,
              4,
              static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF),
              static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF),
              static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF),
              static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF)};
          comm_base_->write_to_device(write_buffer);
        }
        robotstatus_mutex_.unlock();
        // nbytes = write(s, &frame, sizeof(struct can_frame));
      } else {  //! How did you get here?
        throw(-3);
        return;  // TODO: Return error ?
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime));
    }
  }
}

void Pro2ProtocolObject::motors_control_loop(int sleeptime) {
  double linear_vel;
  double angular_vel;
  double rpm1;
  double rpm2;
  double rpm3;
  double rpm4;

  std::chrono::milliseconds time_last =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());
  std::chrono::milliseconds time_from_msg;

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime));
    std::chrono::milliseconds time_now =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
    robotstatus_mutex_.lock();
    linear_vel = robotstatus_.cmd_linear_vel;
    angular_vel = robotstatus_.cmd_angular_vel;
    rpm1 = robotstatus_.motor1_rpm;
    rpm2 = robotstatus_.motor2_rpm;
    rpm3 = robotstatus_.motor3_rpm;
    rpm4 = robotstatus_.motor4_rpm;
    time_from_msg = robotstatus_.cmd_ts;
    robotstatus_mutex_.unlock();
    // float ctrl_update_elapsedtime = (time_now - time_from_msg).count();
    // float pid_update_elapsedtime = (time_now - time_last).count();

    // if (ctrl_update_elapsedtime > CONTROL_LOOP_TIMEOUT_MS_ || estop_) {
    //   robotstatus_mutex_.lock();
    //   motors_speeds_[FRONT_LEFT_MOTOR] = MOTOR_NEUTRAL_;
    //   motors_speeds_[FRONT_RIGHT_MOTOR] = MOTOR_NEUTRAL_;
    //   motors_speeds_[BACK_LEFT_MOTOR] = MOTOR_NEUTRAL_;
    //   motors_speeds_[BACK_RIGHT_MOTOR] = MOTOR_NEUTRAL_;
    //   motor1_control_.reset();
    //   motor2_control_.reset();
    //   motor3_control_.reset();
    //   motor4_control_.reset();
    //   robotstatus_mutex_.unlock();
    //   time_last = time_now;
    //   continue;
    // }

    if (angular_vel == 0) {
      if (linear_vel > 0) {
        angular_vel = trimvalue_;
      } else if (linear_vel < 0) {
        angular_vel = -trimvalue_;
      }
    }
    // !Applying some Skid-steer math
    double left_motors_speed = linear_vel - 0.5 * angular_vel;
    double right_motors_speed = linear_vel + 0.5 * angular_vel;
    // if (left_motors_speed == 0) {
    //   motor1_control_.reset();
    //   motor3_control_.reset();
    // }
    // if (right_motors_speed == 0) {
    //   motor2_control_.reset();
    //   motor4_control_.reset();
    // }
    double motor1_measured_vel = rpm1 / MOTOR_RPM_TO_MPS_RATIO_;
    double motor2_measured_vel = rpm2 / MOTOR_RPM_TO_MPS_RATIO_;
    double motor3_measured_vel = rpm3 / MOTOR_RPM_TO_MPS_RATIO_;
    double motor4_measured_vel = rpm3 / MOTOR_RPM_TO_MPS_RATIO_;
    robotstatus_mutex_.lock();
    // motor speeds in m/s
    // TODO REMOVE
    motors_speeds_[FRONT_LEFT_MOTOR] = left_motors_speed;
    motors_speeds_[BACK_LEFT_MOTOR] = left_motors_speed;
    motors_speeds_[FRONT_RIGHT_MOTOR] = right_motors_speed;
    motors_speeds_[BACK_RIGHT_MOTOR] = right_motors_speed;

    // motors_speeds_[FRONT_LEFT_MOTOR] =
    //     motor1_control_.run(left_motors_speed, motor1_measured_vel,
    //                         pid_update_elapsedtime / 1000, firmware);
    // motors_speeds_[BACK_LEFT_MOTOR] =
    //     motor3_control_.run(left_motors_speed, motor3_measured_vel,
    //                         pid_update_elapsedtime / 1000, firmware);
    // motors_speeds_[FRONT_RIGHT_MOTOR] =
    //     motor2_control_.run(right_motors_speed, motor2_measured_vel,
    //                         pid_update_elapsedtime / 1000, firmware);
    // motors_speeds_[BACK_RIGHT_MOTOR] =
    //     motor4_control_.run(right_motors_speed, motor4_measured_vel,
    //                         pid_update_elapsedtime / 1000, firmware);

    // // Bounding speeds
    // motors_speeds_[FRONT_LEFT_MOTOR] = motor1_control_.boundMotorSpeed(
    //     motors_speeds_[FRONT_LEFT_MOTOR], MOTOR_MAX_, MOTOR_MIN_);
    // motors_speeds_[FRONT_RIGHT_MOTOR] = motor2_control_.boundMotorSpeed(
    //     motors_speeds_[FRONT_RIGHT_MOTOR], MOTOR_MAX_, MOTOR_MIN_);
    // motors_speeds_[BACK_LEFT_MOTOR] = motor3_control_.boundMotorSpeed(
    //     motors_speeds_[BACK_LEFT_MOTOR], MOTOR_MAX_, MOTOR_MIN_);
    // motors_speeds_[BACK_RIGHT_MOTOR] = motor4_control_.boundMotorSpeed(
    //     motors_speeds_[BACK_RIGHT_MOTOR], MOTOR_MAX_, MOTOR_MIN_);
    robotstatus_mutex_.unlock();
    time_last = time_now;
  }
}

}  // namespace RoverRobotics
