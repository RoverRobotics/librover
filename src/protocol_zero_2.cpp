#include "protocol_zero_2.hpp"

namespace RoverRobotics
{

  Zero2ProtocolObject::Zero2ProtocolObject(
      const char *device, std::string new_comm_type,
      Control::robot_motion_mode_t robot_mode, Control::pid_gains pid)
  {
    persistent_params_ =
        std::make_unique<Utilities::PersistentParams>(ROBOT_PARAM_PATH);

    comm_type_ = new_comm_type;
    robot_mode_ = robot_mode;
    robotstatus_ = {0};
    estop_ = false;
    motors_speeds_[LEFT_MOTOR] = MOTOR_NEUTRAL_;
    motors_speeds_[RIGHT_MOTOR] = MOTOR_NEUTRAL_;
    // std::vector<uint32_t> fast_data = {REG_MOTOR_FB_RPM_LEFT,
    //                                    REG_MOTOR_FB_RPM_RIGHT,
    //                                    EncoderInterval_0, EncoderInterval_1};
    // std::vector<uint32_t> slow_data = {
    //     REG_MOTOR_FB_CURRENT_LEFT, REG_MOTOR_FB_CURRENT_RIGHT,
    //     REG_MOTOR_TEMP_LEFT,       REG_MOTOR_TEMP_RIGHT,
    //     REG_MOTOR_CHARGER_STATE,   BuildNO,
    //     BATTERY_VOLTAGE_A};
    pid_ = pid;
    PidGains oldgain = {pid_.kp, pid_.ki, pid_.kd};
    if (robot_mode_ != Control::OPEN_LOOP)
      closed_loop_ = true;
    else
      closed_loop_ = false;
    // motor1_control_ = OdomControl(closed_loop_, oldgain, 1.5, 0);
    // motor2_control_ = OdomControl(closed_loop_, oldgain, 1.5, 0);

    register_comm_base(device);

    // Create a New Thread with 30 mili seconds sleep timer
    // fast_data_write_thread_ =
    //     std::thread([this, fast_data]()
    //                 { this->send_command(30, fast_data); });
    // Create a new Thread with 50 mili seconds sleep timer
    // slow_data_write_thread_ =
    //     std::thread([this, slow_data]() { this->send_command(50, slow_data);
    //     });
    // Create a motor update thread with 30 mili second sleep timer

    /* MUST be done after PID control is constructed */
    load_persistent_params();
    motor_commands_update_thread_ =
        std::thread([this]()
                    { this->motors_control_loop(30); });
    std::cerr << "protocol is running..." << std::endl;
  }

  void Zero2ProtocolObject::load_persistent_params()
  {
    /* trim (aka curvature correction) */
    if (auto param = persistent_params_->read_param("trim"))
    {
      update_drivetrim(param.value());
      std::cout << "Loaded trim from persistent param file: " << param.value()
                << std::endl;
    }
  }

  void Zero2ProtocolObject::update_drivetrim(double value)
  {
    // if (-MAX_CURVATURE_CORRECTION_ < (trimvalue_ + delta) &&
    //     (trimvalue_ + delta) < MAX_CURVATURE_CORRECTION_) {
    //   trimvalue_ += delta;

    //   /* reduce power to right wheels */
    //   if (trimvalue_ >= 0) {
    //     left_trim_ = 1;
    //     right_trim_ = 1 - trimvalue_;
    //   }
    //   /* reduce power to left wheels */
    //   else {
    //     right_trim_ = 1;
    //     left_trim_ = 1 + trimvalue_;
    //   }
    //   skid_control_->setTrim(left_trim_, right_trim_);
    //   std::cout << "writing trim " << trimvalue_ << " to file " << std::endl;
    //   persistent_params_->write_param("trim", trimvalue_);
    // }
  }

  void Zero2ProtocolObject::send_estop(bool estop)
  {
    robotstatus_mutex_.lock();
    estop_ = estop;
    robotstatus_mutex_.unlock();
  }

  robotData Zero2ProtocolObject::status_request()
  {
    return robotstatus_;
  }

  robotData Zero2ProtocolObject::info_request() { return robotstatus_; }

  void Zero2ProtocolObject::set_robot_velocity(double *controlarray)
  {
    robotstatus_mutex_.lock();
    robotstatus_.cmd_linear_vel = controlarray[0];
    robotstatus_.cmd_angular_vel = controlarray[1];
    robotstatus_.cmd_ts = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch());
    robotstatus_mutex_.unlock();
  }

  void Zero2ProtocolObject::motors_control_loop(int sleeptime)
  {
    double linear_vel;
    double angular_vel;
    double rpm1;
    double rpm2;

    std::chrono::milliseconds time_last =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
    std::chrono::milliseconds time_from_msg;

    while (true)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime));
      std::chrono::milliseconds time_now =
          std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::system_clock::now().time_since_epoch());
      robotstatus_mutex_.lock();
      int firmware = robotstatus_.robot_firmware;
      linear_vel = robotstatus_.cmd_linear_vel;
      angular_vel = robotstatus_.cmd_angular_vel;
      rpm1 = robotstatus_.motor1_rpm;
      rpm2 = robotstatus_.motor2_rpm;
      time_from_msg = robotstatus_.cmd_ts;
      robotstatus_mutex_.unlock();
      float ctrl_update_elapsedtime = (time_now - time_from_msg).count();
      float pid_update_elapsedtime = (time_now - time_last).count();

      if (ctrl_update_elapsedtime > CONTROL_LOOP_TIMEOUT_MS_ || estop_)
      {
        robotstatus_mutex_.lock();
        motors_speeds_[LEFT_MOTOR] = MOTOR_NEUTRAL_;
        motors_speeds_[RIGHT_MOTOR] = MOTOR_NEUTRAL_;
        motor1_control_.reset();
        motor2_control_.reset();
        robotstatus_mutex_.unlock();
        send_motors_commands();
        time_last = time_now;
        continue;
      }

      if (angular_vel == 0)
      {
        if (linear_vel > 0)
        {
          angular_vel = trimvalue_;
        }
        else if (linear_vel < 0)
        {
          angular_vel = -trimvalue_;
        }
      }
      // !Applying some Skid-steer math
      double motor1_vel = linear_vel - 0.5 * angular_vel;
      double motor2_vel = linear_vel + 0.5 * angular_vel;
      if (motor1_vel == 0)
        motor1_control_.reset();
      if (motor2_vel == 0)
        motor2_control_.reset();

      double motor1_measured_vel = rpm1 / MOTOR_RPM_TO_MPS_RATIO_;
      double motor2_measured_vel = rpm2 / MOTOR_RPM_TO_MPS_RATIO_;
      robotstatus_mutex_.lock();
      // motor speeds in m/s
      motors_speeds_[LEFT_MOTOR] =
          motor1_control_.run(motor1_vel, motor1_measured_vel,
                              pid_update_elapsedtime / 1000, firmware);
      motors_speeds_[RIGHT_MOTOR] =
          motor2_control_.run(motor2_vel, motor2_measured_vel,
                              pid_update_elapsedtime / 1000, firmware);
      robotstatus_mutex_.unlock();
      send_motors_commands();
      time_last = time_now;
    }
  }
  void Zero2ProtocolObject::unpack_comm_response(std::vector<unsigned char> robotmsg)
  {
    static std::vector<uint32_t> msgqueue;
    robotstatus_mutex_.lock();
    msgqueue.insert(msgqueue.end(), robotmsg.begin(),
                    robotmsg.end()); // insert robotmsg to msg list
    // ! Delete bytes until valid start byte is found
    if ((unsigned char)msgqueue[0] != startbyte_ &&
        msgqueue.size() > RECEIVE_MSG_LEN_)
    {
      int startbyte_index = 0;
      // !Did not find valid start byte in buffer
      while (msgqueue[startbyte_index] != startbyte_ &&
             startbyte_index < msgqueue.size())
        startbyte_index++;
      if (startbyte_index >= msgqueue.size())
      {
        msgqueue.clear();
        return;
      }
      else
      {
        // !Reconstruct the vector so that the start byte is at the 0 position
        std::vector<uint32_t> temp;
        for (int x = startbyte_index; x < msgqueue.size(); x++)
        {
          temp.push_back(msgqueue[x]);
        }
        msgqueue.clear();
        msgqueue.resize(0);
        msgqueue = temp;
        temp.clear();
      }
    }
    if ((unsigned char)msgqueue[0] == startbyte_ &&
        msgqueue.size() >= RECEIVE_MSG_LEN_)
    { // if valid start byte
      unsigned char start_byte_read, data1, data2, dataNO, checksum,
          read_checksum;
      start_byte_read = (unsigned char)msgqueue[0];
      dataNO = (unsigned char)msgqueue[1];
      data1 = (unsigned char)msgqueue[2];
      data2 = (unsigned char)msgqueue[3];
      checksum = 255 - (dataNO + data1 + data2) % 255;
      read_checksum = (unsigned char)msgqueue[4];
      if (checksum == read_checksum)
      { // verify checksum
        int16_t b = (data1 << 8) + data2;
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
            0.5 * (robotstatus_.motor1_rpm / MOTOR_RPM_TO_MPS_RATIO_ +
                   robotstatus_.motor2_rpm / MOTOR_RPM_TO_MPS_RATIO_);

        robotstatus_.angular_vel =
            ((robotstatus_.motor1_rpm / MOTOR_RPM_TO_MPS_RATIO_) -
             (robotstatus_.motor2_rpm / MOTOR_RPM_TO_MPS_RATIO_)) *
            odom_angular_coef_ * odom_traction_factor_;

        std::vector<uint32_t> temp;
        // !Remove processed msg from queue
        for (int x = RECEIVE_MSG_LEN_; x < msgqueue.size(); x++)
        {
          temp.push_back(msgqueue[x]);
        }
        msgqueue.clear();
        msgqueue.resize(0);
        msgqueue = temp;
        temp.clear();
      }
      else
      { // !Found start byte but the msg contents were invalid, throw away
        // broken message
        std::vector<uint32_t> temp;
        for (int x = 1; x < msgqueue.size(); x++)
        {
          temp.push_back(msgqueue[x]);
        }
        msgqueue.clear();
        msgqueue.resize(0);
        msgqueue = temp;
        temp.clear();
      }
    }
    else
    {
      // !ran out of data; waiting for more
    }
    robotstatus_mutex_.unlock();
  }

  bool Zero2ProtocolObject::is_connected() { return comm_base_->is_connected(); }

  int Zero2ProtocolObject::cycle_robot_mode()
  {
    // TODO
    return 0;
  }
  void Zero2ProtocolObject::register_comm_base(const char *device)
  {
    if (comm_type_ == "serial")
    {
      std::vector<uint32_t> setting;
      setting.push_back(termios_baud_code_);
      setting.push_back(RECEIVE_MSG_LEN_);
      try
      {
        comm_base_ = std::make_unique<CommSerial>(
            device, [this](std::vector<unsigned char> c)
            { unpack_comm_response(c); },
            setting);
      }
      catch (int i)
      {
        std::cerr << "error";
        throw(i);
      }
    }
    else
    { // not supported device
      std::cerr << "not supported";
      throw(-2);
    }
  }

  void Zero2ProtocolObject::send_command(int sleeptime,
                                         std::vector<uint32_t> datalist)
  {
    while (true)
    {
      for (int x : datalist)
      {
        if (comm_type_ == "serial")
        {
          //send motor one duty
          robotstatus_mutex_.lock();
          //WIP
          int32_t v = static_cast<int32_t>(motors_speeds_[LEFT_MOTOR] * 100000.0);
          std::vector<unsigned char> write_buffer = {
              PAYLOAD_BYTE_SIZE_,
              MSG_SIZE_,
              COMM_SET_DUTY,
              static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF),
              static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF),
              static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF),
              static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF),

          };

          // unsigned char *payload;
          unsigned char *payload = write_buffer.data();

          // std::copy(write_buffer.begin(), write_buffer.end(), payload);
          // payload = &write_buffer[0];
          uint16_t crc = crc16(payload, write_buffer[1]);
          write_buffer.push_back(static_cast<uint8_t>(crc >> 8));
          write_buffer.push_back(static_cast<uint8_t>(crc & 0xFF));
          write_buffer.push_back(STOP_BYTE_);
          write_buffer.push_back(STOP_BYTE2_);
          comm_base_->write_to_device(write_buffer);
          robotstatus_mutex_.unlock();
        }
        else if (comm_type_ == "can")
        {
          return;
        }
        else
        {         //! How did you get here?
          return; // TODO: Return error ?
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime));
      }
    }
  }

  void Zero2ProtocolObject::send_motors_commands()
  {
    robotstatus_mutex_.lock();
    //WIP
    int32_t v = static_cast<int32_t>(motors_speeds_[LEFT_MOTOR] * 100000.0);
    std::vector<unsigned char> write_buffer = {
        PAYLOAD_BYTE_SIZE_,
        MSG_SIZE_,
        COMM_SET_DUTY,
        static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF),
        static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF),
        static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF),
        static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF)};
    unsigned char *payload = write_buffer.data();
    // std::copy(write_buffer.begin(), write_buffer.end(), payload);
    // payload = &write_buffer[0];
    uint16_t crc = crc16(payload, write_buffer[1]);
    write_buffer.push_back(static_cast<uint8_t>(crc >> 8));
    write_buffer.push_back(static_cast<uint8_t>(crc & 0xFF));
    write_buffer.push_back(STOP_BYTE_);
    write_buffer.push_back(STOP_BYTE2_);
    comm_base_->write_to_device(write_buffer);
    robotstatus_mutex_.unlock();
    write_buffer.clear();
    robotstatus_mutex_.lock();
    //WIP
    v = static_cast<int32_t>(motors_speeds_[RIGHT_MOTOR] * 100000.0);
    write_buffer = {PAYLOAD_BYTE_SIZE_,
                    FORWARD_MSG_SIZE_, COMM_CAN_FORWARD,
                    RIGHT_MOTOR,
                    COMM_SET_DUTY,
                    static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF),
                    static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF),
                    static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF),
                    static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF)};
    unsigned char *payloadptr;
    unsigned char payload2[7];
    payload2[0] = COMM_CAN_FORWARD;
    payload2[1] = RIGHT_MOTOR;
    payload2[2] = COMM_SET_DUTY;
    payload2[3] = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
    payload2[4] = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
    payload2[5] = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
    payload2[6] = static_cast<uint8_t>((static_cast<uint32_t>(v)) & 0xFF);
    payloadptr = payload2;
    crc = crc16(payload2, FORWARD_MSG_SIZE_);
    write_buffer.push_back(static_cast<uint8_t>(crc >> 8));
    write_buffer.push_back(static_cast<uint8_t>(crc & 0xFF));
    write_buffer.push_back(STOP_BYTE_);
    write_buffer.push_back(STOP_BYTE2_);
    comm_base_->write_to_device(write_buffer);
    robotstatus_mutex_.unlock();
  }
  unsigned short Zero2ProtocolObject::crc16(unsigned char *buf, unsigned int len)
  {
    unsigned int i;
    unsigned short cksum = 0;
    for (i = 0; i < len; i++)
    {
      cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
    }
    return cksum;
  }
} // namespace RoverRobotics

// *(payload_.first + 1) = static_cast<uint8_t>(canid);
// *(payload_.first + 2) = static_cast<uint8_t>(COMM_GET_VALUES);
// VescFrame::CRC crc_calc;
// crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
// uint16_t crc = crc_calc.checksum();
// *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
// *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);