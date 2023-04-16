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
  

  // Read CANCTRL register
  unsigned char spi_read_canctrl[] = {
    MCP_CMD_READ,
    MCP_REG_CANCNTRL,
    0x00
  };
  ftdi_write_data(ftdi, spi_read_canctrl, 3);
  printf("Wrote read command for CANCTRL\n");
  unsigned char spi_read_buffer[3];
  ftdi_read_data(ftdi, spi_read_buffer, 3);
  printf("CANCTRL Register: 0x%x\n", spi_read_buffer[2]);

  /*
  // configure SPI bus
  unsigned char config[] = {
  0x80,     // disable divide by 5 (0x80 | 0x08)
  0x8A,     // disable adaptive clocking (0x8A)
  0x86,     // enable 3-phase data clocking (0x86)
  0x02,     // clock divisor (0x02 = 1 MHz)
  0x00,     // delay between chip select and data (in microseconds)
  0x00,     // delay between successive data bytes (in microseconds)
  0x00      // mode flags (CPOL=0, CPHA=0)
  };

  // Send configuration to FTDI device
  ret = ftdi_write_data(ftdi, config, sizeof(config));
  if (ret < 0) {
    fprintf(stderr, "Unable to send SPI configuration (%s)\n", ftdi_get_error_string(ftdi));
    ftdi_usb_close(ftdi);
    ftdi_free(ftdi);
    throw(OPEN_DEVICE_FAIL);
  }
  */
  // start read thread
  
  Can_read_thread_ = std::thread(
      [this, parsefunction]() { this->read_device_loop(parsefunction); });
  
}

void CommCanSPI::write_to_device(std::vector<uint8_t> msg) {
  std::cout << "Expected CAN message: ";
  for (int i = 0; i < CAN_MSG_SIZE_; i++) {
    std::cout << std::hex << static_cast<int>(msg[i]) << " ";
  }
  std::cout << std::endl;
  Can_write_mutex_.lock();
  if (msg.size() == CAN_MSG_SIZE_) {
    // convert msg to spi frame
    int spi_msg_size = msg.size() + 3; // msg size = 9 + 3 bytes for SPI write
    unsigned char write_buffer[] = {
      MCP_CMD_WRITE, // Write to MCP
      0x01, // Address high Byte
      0x23, // Address Low Byte
      msg[0],
      msg[1],
      msg[2],
      msg[3],
      msg[4],
      msg[5],
      msg[6],
      msg[7],
      msg[8]
    };

    ftdi_write_data(ftdi, write_buffer, spi_msg_size);
  }
  Can_write_mutex_.unlock();
}

void CommCanSPI::read_device_loop(std::function<void(std::vector<uint8_t>)> parsefunction) {
  std::chrono::milliseconds time_last =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());
  unsigned char read_buffer[14];
  while (true) {
    int num_bytes = ftdi_read_data(ftdi, read_buffer, 14);
    std::chrono::milliseconds time_now =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
    if (num_bytes <= 0) {
      if ((time_now - time_last).count() > TIMEOUT_MS_) {
        is_connected_ = false;
        ftdi_usb_close(ftdi);
      }
      continue;
    }
    //printf("Number of Bytes Read in Device Loop: %d\n", num_bytes);
    is_connected_ = true;
    time_last = time_now;
    std::vector<uint8_t> msg;

    for (int i = 0; i < num_bytes; i++) {
      //printf("Read Byte[%d]: 0x%x\n", i, read_buffer[i]);
      msg.push_back(read_buffer[i]);
    }
    
    parsefunction(msg);
    //msg.clear();
    
  }
}


bool CommCanSPI::is_connected() { return (is_connected_); }

}  // namespace RoverRobotics