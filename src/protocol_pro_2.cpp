#include "protocol_pro_2.hpp"

#include <iomanip>
#include <iostream>
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
  pid_ = pid;
  register_comm_base(device);
  // TODO
  // std::vector<uint32_t> fast_data = {REG_MOTOR_FB_RPM_LEFT,
  //                                    REG_MOTOR_FB_RPM_RIGHT,
  //                                    EncoderInterval_0, EncoderInterval_1};
  // Create a New Thread with 30 mili seconds sleep timer
  // fast_data_write_thread_ =
  //     std::thread([this, fast_data]() { this->send_command(30, fast_data);
  //     });
  // // Create a new Thread with 50 mili seconds sleep timer
  slow_data_write_thread_ = std::thread([this]() { this->send_command(30); });
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

void Pro2ProtocolObject::set_robot_velocity(double *control_array) {
  robotstatus_mutex_.lock();
  robotstatus_.cmd_linear_vel = control_array[0];
  robotstatus_.cmd_angular_vel = control_array[1];
  // robotstatus_.cmd_ts =
  // std::chrono::duration_cast<std::chrono::milliseconds>(
  //     std::chrono::system_clock::now().time_since_epoch());
  robotstatus_mutex_.unlock();
}
// REMOVE?
// void Pro2ProtocolObject::send_speed(double *control_array) {
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
//         double linear_rate = control_array[0];
//         double turn_rate = control_array[1];
//         double flipper_rate = control_array[2];
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
  if (comm_type_ == "can") {
    // std::cerr << std::bitset<32>(robotmsg[0]) << " ";
    //  std::cerr << test << " " << std::hex << robotmsg[0];
    // for (int i = 0; i < sizeof(robotmsg); i++) {
    //   std::cerr << std::hex << robotmsg[i] << " ";
    // }
    // std::cerr << std::endl;
    if ((robotmsg[0] == 0x001B)) {
    }
    // if ((robotmsg[0] & 0x900) == 0x900) {
    if ((robotmsg[0] & 0xFFFFFF00) == 0x80000900) {
      // for (auto a : robotmsg) {
      //   std::cerr << std::hex << a << " ";
      // }
      // std::cerr << std::endl;
      int vesc_id = robotmsg[0] & 0xFF;
      int32_t rpm_scaled = ((uint8_t)robotmsg[2] << 24) |
                           ((uint8_t)robotmsg[3] << 16) |
                           ((uint8_t)robotmsg[4] << 8) | ((uint8_t)robotmsg[5]);
      int16_t current_scaled =
          ((uint8_t)robotmsg[6] << 8) | ((uint8_t)robotmsg[7]);
      int16_t duty_scaled =
          (((uint8_t)robotmsg[8] << 8) | ((uint8_t)robotmsg[9]));

      float rpm = (float)rpm_scaled / 1000 * 60;
      float current = (float)current_scaled / 10;
      float duty = (float)duty_scaled / 10;
      // std::cerr << "Vesc ID " << vesc_id << "RPM " << rpm << " Current "
      //           << current << " Duty " << duty << std::endl;
      if (vesc_id == 0) {
        robotstatus_.motor1_rpm = rpm;
        robotstatus_.motor1_id = vesc_id;
        robotstatus_.motor1_current = current;
        // robotstatus_.motor1_duty = duty;
      } else if (vesc_id == 1) {
        robotstatus_.motor2_rpm = rpm;
        robotstatus_.motor2_id = vesc_id;
        robotstatus_.motor2_current = current;
        // robotstatus_.motor2_duty = duty;
      } else if (vesc_id == 2) {
        robotstatus_.motor3_rpm = rpm;
        robotstatus_.motor3_id = vesc_id;
        robotstatus_.motor3_current = current;
        // robotstatus_.motor3_duty = duty;
      } else if (vesc_id == 3) {
        robotstatus_.motor4_rpm = rpm;
        robotstatus_.motor4_id = vesc_id;
        robotstatus_.motor4_current = current;
        // robotstatus_.motor4_duty = duty;
      } else {
        return;
      }
    }
  }
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

void Pro2ProtocolObject::send_command(int sleeptime) {
  while (true) {
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

void Pro2ProtocolObject::motors_control_loop(int sleeptime) {
  float linear_vel, angular_vel, rpm_FL, rpm_FR, rpm_BL, rpm_BR;
  Control::robot_geometry robot_geometry = {0.205, 0.265, .01651, 0, 0};
  Control::pid_gains pid_gains = {0.003, 0.0, 0.0010};
  float motor_max_duty = .95;
  Control::SkidRobotMotionController skid_control =
      Control::SkidRobotMotionController(
          Control::TRACTION_CONTROL, robot_geometry, pid_gains, motor_max_duty);
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
    // get data from robot
    linear_vel = robotstatus_.cmd_linear_vel;
    angular_vel = robotstatus_.cmd_angular_vel;
    rpm_FL = robotstatus_.motor1_rpm;
    rpm_FR = robotstatus_.motor2_rpm;
    rpm_BL = robotstatus_.motor3_rpm;
    rpm_BR = robotstatus_.motor4_rpm;
    // time_from_msg = robotstatus_.cmd_ts;
    robotstatus_mutex_.unlock();
    // generate new duty for robot
    // if robot havn't reach timeout
    // Skid steer math to generate motor speeds
    // double left_motors_speed = linear_vel - 0.5 * angular_vel;
    // double right_motors_speed = linear_vel + 0.5 * angular_vel;
    auto duty_cycles =
        skid_control.runMotionControl({linear_vel, angular_vel}, {0, 0, 0, 0},
                                      {rpm_FL, rpm_FR, rpm_BL, rpm_BR});
    robotstatus_mutex_.lock();
    motors_speeds_[FRONT_LEFT_MOTOR] = duty_cycles.fl;
    motors_speeds_[FRONT_RIGHT_MOTOR] = duty_cycles.fr;
    motors_speeds_[BACK_LEFT_MOTOR] = duty_cycles.rl;
    motors_speeds_[BACK_RIGHT_MOTOR] = duty_cycles.rr;
    robotstatus_mutex_.unlock();

    // end timeout
    time_last = time_now;
  }
}

}  // namespace RoverRobotics
