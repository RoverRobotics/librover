#include "comm_can_spi.hpp"

namespace RoverRobotics {
CommCanSPI::CommCanSPI(const char *device, std::function<void(std::vector<uint8_t>)> parsefunction, std::vector<uint8_t> setting) : is_connected_(false) {
  
  // Create mpsse context and check if successful
  struct mpsse_context ftdic;
    int ret;
    if ((ret = ftdi_init(&(ftdic.ftdi))) < 0) {
        fprintf(stderr, "ftdi_init failed: %d\n", ret);
        throw(-1);
    }
    if ((ret = ftdi_usb_open(&(ftdic.ftdi), 0x0403, 0x6010)) < 0) {
        fprintf(stderr, "ftdi_usb_open_desc failed: %d (%s)\n", ret, ftdi_get_error_string(&(ftdic.ftdi)));
        ftdi_deinit(&(ftdic.ftdi));
        throw(-1);
    }
    // MPSSE device is open, do something with it
    ftdi_usb_close(&(ftdic.ftdi));
    ftdi_deinit(&(ftdic.ftdi));
    throw(-1);
  // start read thread
  /*
  Can_read_thread_ = std::thread(
      [this, parsefunction, mpsse]() { this->read_device_loop(parsefunction); });
      */
}

void CommCanSPI::write_to_device(std::vector<uint8_t> msg) {
  /*
  Can_write_mutex_.lock();
  if (msg.size() == CAN_MSG_SIZE_) {
    // convert msg to frame
    frame.can_id = static_cast<uint32_t>((msg[0] << 24) + (msg[1] << 16) +
                                         (msg[2] << 8) + msg[3]);
    frame.can_dlc = msg[4];
    frame.data[0] = msg[5];
    frame.data[1] = msg[6];
    frame.data[2] = msg[7];
    frame.data[3] = msg[8];
    write(fd, &frame, sizeof(struct can_frame));
  }
  Can_write_mutex_.unlock();
  */
}

void CommCanSPI::read_device_loop(
    std::function<void(std::vector<uint8_t>)> parsefunction) {
  /*
  std::chrono::milliseconds time_last =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());
  while (true) {
    int num_bytes = read(fd, &robot_frame, sizeof(robot_frame));
    std::chrono::milliseconds time_now =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
    if (num_bytes <= 0) {
      if ((time_now - time_last).count() > TIMEOUT_MS_) {
        is_connected_ = false;
      }
      continue;
    }
    is_connected_ = true;
    time_last = time_now;
    std::vector<uint8_t> msg;

    msg.push_back(robot_frame.can_id >> 24);
    msg.push_back(robot_frame.can_id >> 16);
    msg.push_back(robot_frame.can_id >> 8);
    msg.push_back(robot_frame.can_id);
    msg.push_back(robot_frame.can_dlc);
    for (int i = 0; i < sizeof(robot_frame.data); i++) {
      msg.push_back(robot_frame.data[i]);
    }
    parsefunction(msg);
    msg.clear();
    
  }
  */
}

bool CommCanSPI::is_connected() { return (is_connected_); }

}  // namespace RoverRobotics