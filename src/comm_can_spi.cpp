#include "comm_can_spi.hpp"
namespace RoverRobotics {
CommCanSPI::CommCanSPI(const char *device, std::function<void(std::vector<uint8_t>)> parsefunction, std::vector<uint8_t> setting) : is_connected_(false) {
  // FTDI Setup for MPSSE Mode
  char* data = NULL;

  char read_canctrl_cmd[] = {
    MCP_CMD_READ,
    MCP_REG_CANCNTRL
  };
  char read_canstat_cmd[] = {
    MCP_CMD_READ,
    0b00001110
  };
  char conf_can_cmd[] = {
    MCP_CMD_WRITE,
    0b00101000,
    0x05,
    0xB5,
    0x40
  };

  char conf_canctrl[] = {
    MCP_CMD_WRITE,
    0b00001111,
    0b10011000
  };

  char read_txb0_cmd[] = {
      MCP_CMD_READ,
      0x30
    };

  char send_one_msg[] = {
    MCP_CMD_WRITE,
    0x31,
    0x80,
    0x00,
    0x01,
    0x01,
    0x04,
    0x01,
    0x02,
    0x03,
    0x04
  };

  char conf_no_int[] = {
    MCP_CMD_WRITE,
    0b00101011,
    0b00000100
  };

  if(ftdi = OpenIndex(0x0403, 0x6010, SPI0, TEN_MHZ, MSB, IFACE_A, NULL, NULL, 0)){
    printf("%s opened at %dHz (SPI Mode 0)\n", GetDescription(ftdi), GetClock(ftdi));

    Start(ftdi);
    Write(ftdi, read_canctrl_cmd, sizeof(read_canctrl_cmd));
    data = Read(ftdi, 1);
    Stop(ftdi);

    printf("CANCTRL Data Received: 0x%02x | ", data[0]);
    for(int i = 0; i < 8; i++){
      printf("%d", ((data[0] >> (7-i)) & 1));
    }
    printf("\n");

    Start(ftdi);
    Write(ftdi, read_canstat_cmd, sizeof(read_canstat_cmd));
    data = Read(ftdi, 1);
    Stop(ftdi);

    printf("CANSTAT Data Received: 0x%02x | ", data[0]);
    for(int i = 0; i < 8; i++){
      printf("%d", ((data[0] >> (7-i)) & 1));
    }
    printf("\n");


    printf("Configuring CANCTRL and CNF[3:1]\n");
    Start(ftdi);
    Write(ftdi, conf_canctrl, sizeof(conf_canctrl));
    Stop(ftdi);

    Start(ftdi);
    Write(ftdi, conf_can_cmd, sizeof(conf_can_cmd));
    Stop(ftdi);

    Start(ftdi);
    Write(ftdi, "\x03\x28", 2);
    data = Read(ftdi, 3);
    Stop(ftdi);

    printf("CNF[3:1]: 0x%02x, 0x%02x, 0x%02x\n", data[0], data[1], data[2]);
    printf("Setting loopback mode...\n");
    Start(ftdi);
    Write(ftdi, "\x02\x0F\x68", 3);
    Stop(ftdi);

    Start(ftdi);
    Write(ftdi, read_canctrl_cmd, sizeof(read_canctrl_cmd));
    data = Read(ftdi, 1);
    Stop(ftdi);

    printf("CANCTRL is now: 0x%02x | ", data[0]);
    for(int i = 0; i < 8; i++){
      printf("%d", ((data[0] >> (7-i)) & 1));
    }
    printf("\n");


    printf("Sending one msg of data...\n");
    Start(ftdi);
    Write(ftdi, send_one_msg, sizeof(send_one_msg));
    Stop(ftdi);

    printf("Reading Buffer 0...\n");
    Start(ftdi);
    Write(ftdi, "\x03\x31", 2);
    data = Read(ftdi, 9);
    Stop(ftdi);

    std::cout << "Buffer 0: ";
    for (int i = 0; i < CAN_MSG_SIZE_; i++) {
      printf("0x%02x ", data[i]);
    }
    std::cout << std::endl;

    Start(ftdi);
    Write(ftdi, conf_no_int, sizeof(conf_no_int));
    Stop(ftdi);

    Start(ftdi);
    Write(ftdi, "\x30\x0B", 2);
    Stop(ftdi);


    Start(ftdi);
    Write(ftdi, "\x03\x2c", 2);
    data = Read(ftdi, 1);
    Stop(ftdi);

    printf("CANTINF is now: 0x%02x | ", data[0]);
    for(int i = 0; i < 8; i++){
      printf("%d", ((data[0] >> (7-i)) & 1));
    }
    printf("\n");

    sleep(0.05);

    printf("Reading one msg of data...\n");
    Start(ftdi);
    Write(ftdi, "\x03\x61", 2);
    data = Read(ftdi, 9);
    Stop(ftdi);

    std::cout << "Received CAN message: ";
    for (int i = 0; i < CAN_MSG_SIZE_; i++) {
      printf("0x%02x ", data[i]);
    }
    std::cout << std::endl;

  }
  else {
    printf("Failed to initialize MPSSE: %s\n", ErrorString(ftdi));
  }

  Close(ftdi);
  // start read thread
  /*
  Can_read_thread_ = std::thread(
      [this, parsefunction]() { this->read_device_loop(parsefunction); });
  */
}

void CommCanSPI::write_to_device(std::vector<uint8_t> msg) {
  std::cout << "Expected CAN message: ";
  for (int i = 0; i < CAN_MSG_SIZE_; i++) {
    printf("0x%02x ", msg[i]);
  }
  std::cout << std::endl;
  Can_write_mutex_.lock();
  if (msg.size() == CAN_MSG_SIZE_) {
    // convert msg to spi frame
    char read_txb0_cmd[] = {
      MCP_CMD_READ,
      0b00001110
    };

    char transmit_tx_buffer[] = {
      MCP_CMD_WRITE,
      0x30,
      0x0B
    };

    char* data = NULL;

    char load_tx_buffer[] = {
      MCP_CMD_WRITE,
      0x31,
      msg[0],
      msg[1],
      msg[2],
      msg[3],
      msg[4],
      msg[5],
      msg[6],
      msg[7],
      msg[8],
    };

    Start(ftdi);
    Write(ftdi, load_tx_buffer, sizeof(load_tx_buffer));
    Stop(ftdi);

    Start(ftdi);
    Write(ftdi, transmit_tx_buffer, sizeof(transmit_tx_buffer));
    Stop(ftdi);

    Close(ftdi);
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
