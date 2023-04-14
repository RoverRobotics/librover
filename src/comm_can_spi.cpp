#include "comm_can_spi.hpp"

namespace RoverRobotics {
CommCanSPI::CommCanSPI(const char *device, std::function<void(std::vector<uint8_t>)> parsefunction, std::vector<uint8_t> setting) : is_connected_(false) {
  
  // Create mpsse context and check if successful
  struct mpsse_context *mpsse = MPSSE(SPI0, ONE_MHZ, MSB);
  if (!mpsse) { 
    std::cerr << "Failed to initialize MPSSE context" << std::endl;
    throw(-1);
  }
  
  // Open connection with device (For now: vid 0403, pid 6010, bus 001, device 004, spi freq 1MHz)
  if (ftdi_usb_open_string(&(mpsse->ftdi), device) != 1){
    std::cerr << "Failed to open " << device;
    throw(-1);
  }

  std::cout << "Opened " << mpsse->ftdi.usb_dev;

  // start read thread
  Can_read_thread_ = std::thread(
      [this, parsefunction, mpsse]() { this->read_device_loop(parsefunction); });
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