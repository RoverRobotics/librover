#include "comm_can_spi.hpp"

namespace RoverRobotics {
CommCanSPI::CommCanSPI(const char *device, std::function<void(std::vector<uint8_t>)> parsefunction, std::vector<uint8_t> setting) : is_connected_(false) {
  // Local Variables
  int ret;
  struct ftdi_device_list *devlist;

  // Create FTDI Context
  if ((ftdi = ftdi_new()) == 0) { 
    fprintf(stderr, "Creating FTDI context failed [ftdi_new()]\n");
    throw(FTDI_CREATION_FAIL); 
  }

  // Open FTDI Device and throw an error if it fails
  if ((ret = ftdi_usb_open_desc_index(ftdi, 0x0403, 0x6010, NULL, NULL, 1) < 0)){
    fprintf(stderr, "Unable to open ftdi device %d (%s)\n", ret, ftdi_get_error_string(ftdi));
    ftdi_free(ftdi);
    throw(OPEN_DEVICE_FAIL);
  }

  // Select Interface A for can
  printf("Selecting Channel A: %i\n", ftdi_set_interface(ftdi, INTERFACE_A));

  // Set timeout
  printf("Setting the latency timeout value: %i\n", ftdi_set_latency_timer(ftdi, 2));

  // Enable MPSSE mode
  printf("Setting to MPSSE Mode: %i\n", ftdi_set_bitmode(ftdi, 0, BITMODE_MPSSE));
  
  // configure SPI bus
  unsigned char config[] = {0x8a, 0x97, 0x0b, 0x00, 0x00};
  // start read thread
  /*
  Can_read_thread_ = std::thread(
      [this, parsefunction]() { this->read_device_loop(parsefunction); });
  */
}

void CommCanSPI::write_to_device(std::vector<uint8_t> msg) {
  /*
  Can_write_mutex_.lock();
  if (msg.size() == CAN_MSG_SIZE_) {
    // convert msg to frame
    frame.can_id = static_cast<uint32_t>((msg[0] << 24) + (msg[1] << 16) + (msg[2] << 8) + msg[3]);
    
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

void CommCanSPI::read_device_loop(std::function<void(std::vector<uint8_t>)> parsefunction) {
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

/*
int CommCanSPI::convert_frame_to_spi(const can_frame& frame, uint8_t* buf){
  int len = 0;
  buf[len++] = frame.can_id >> 24;
}
*/

bool CommCanSPI::is_connected() { return (is_connected_); }

}  // namespace RoverRobotics