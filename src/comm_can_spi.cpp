#include "comm_can_spi.hpp"
namespace RoverRobotics {
CommCanSPI::CommCanSPI(const char *device, std::function<void(std::vector<uint8_t>)> parsefunction, std::vector<uint8_t> setting) : is_connected_(false) {
  // FTDI Setup for MPSSE Mode
  char* data = NULL;

  char read_canctrl_cmd[] = {
    MCP_CMD_READ,
    MCP_REG_CANCNTRL
  };

  if(ftdi = OpenIndex(0x0403, 0x6010, SPI0, TEN_MHZ, MSB, IFACE_A, NULL, NULL, 0)){
    printf("%s opened at %dHz (SPI Mode 0)", GetDescription(ftdi), GetClock(ftdi));

    Start(ftdi);
    Write(ftdi, read_canctrl_cmd, sizeof(read_canctrl_cmd));
    data = Read(ftdi, 1);
    Stop(ftdi);

    printf("CANCTRL Data Received: 0x%02x\n", data[0]);
  }
  else {
    printf("Failed to initialize MPSSE: %s\n", ErrorString(ftdi));
  }

  Close(ftdi);
  // start read thread
  Can_read_thread_ = std::thread(
      [this, parsefunction]() { this->read_device_loop(parsefunction); });
}

void CommCanSPI::write_to_device(std::vector<uint8_t> msg) {
  // std::cout << "Expected CAN message: ";
  // for (int i = 0; i < CAN_MSG_SIZE_; i++) {
  //   std::cout << std::hex << static_cast<int>(msg[i]) << " ";
  // }
  // std::cout << std::endl;
  Can_write_mutex_.lock();
  if (msg.size() == CAN_MSG_SIZE_) {
    // convert msg to spi frame
    
    int spi_msg_size = msg.size() + 3; // msg size = 9 + 3 bytes for SPI write
    char load_tx_buffer[] = {
      MCP_CMD_WRITE,
      0x31,
      msg[0],
      0x32,
      msg[1],
      0x33,
      msg[2],
      0x34,
      msg[3],
      0x35,
      msg[4],
      0x36,
      msg[5],
      0x37,
      msg[6],
      0x38,
      msg[7],
      0x39,
      msg[8],
    };

    Start(ftdi);
    Write(ftdi, load_tx_buffer, sizeof(load_tx_buffer));
    Stop(ftdi);

    char transmit_tx_buffer[] = {
      0x30,
      0x08
    };

    Start(ftdi);
    Write(ftdi, transmit_tx_buffer, sizeof(transmit_tx_buffer));
    Stop(ftdi);
  }
  Can_write_mutex_.unlock();
}

void CommCanSPI::read_device_loop(std::function<void(std::vector<uint8_t>)> parsefunction) {
  std::chrono::milliseconds time_last =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());
  char* read_buffer;
  char read_cmd[1]{
    MCP_CMD_READ
  };
  while (true) {
    /*
    Start(ftdi);
    Write(ftdi, read_cmd, sizeof(read_cmd));
    read_buffer = Read(ftdi, 14);
    Stop(ftdi);
    */
    //int num_bytes = ftdi_read_data(ftdi, read_buffer, 14);
    int num_bytes = 0; // to implement read
    std::chrono::milliseconds time_now =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
    if (num_bytes <= 0) {
      if ((time_now - time_last).count() > TIMEOUT_MS_) {
        is_connected_ = false;
        //ftdi_usb_close(ftdi);
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
    
    try{
      parsefunction(msg);
      is_connected_ = true;
    } catch(int i){
      if (i == -4){
        //printf("Not a valid RPM packet. Connection = False\n");
        is_connected_ = false;
      }
    }
    //msg.clear();
    
  }
}


bool CommCanSPI::is_connected() { return (is_connected_); }

}  // namespace RoverRobotics
