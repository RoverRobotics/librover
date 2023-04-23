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
    0x04,
    0xBA,
    0x01
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
    0x01,
    0x23,
    0x1f,
    0x00,
    0x04,
    0xaa,
    0xbb,
    0xcc,
    0xdd
  };

  char conf_no_int[] = {
    MCP_CMD_WRITE,
    0b00101011,
    0b00000100
  };

  if(ftdi = OpenIndex(0x0403, 0x6010, SPI0, TEN_MHZ, MSB, IFACE_A, NULL, NULL, 0)){
    printf("%s opened at %dHz (SPI Mode 0)\n", GetDescription(ftdi), GetClock(ftdi));
    printf("Setting default settings...\n");
    Start(ftdi);
    Write(ftdi, "\xC0", 1);
    Stop(ftdi);
    
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
    printf("Setting normal one shot mode...\n");
    Start(ftdi);
    Write(ftdi, "\x02\x0F\x08", 3);
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

    /*
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
    Write(ftdi, "\x81", 2);
    Stop(ftdi);
    */

    Start(ftdi);
    Write(ftdi, "\x03\x30", 2);
    data = Read(ftdi, 1);
    Stop(ftdi);

    printf("TXB0 is now: 0x%02x | ", data[0]);
    for(int i = 0; i < 8; i++){
      printf("%d", ((data[0] >> (7-i)) & 1));
    }
    printf("\n");

    sleep(0.05);

    printf("Reading TEC...\n");
    Start(ftdi);
    Write(ftdi, "\x03\x1C", 2);
    data = Read(ftdi, 1);
    Stop(ftdi);

    printf("TEC is now: 0x%02x | ", data[0]);
    for(int i = 0; i < 8; i++){
      printf("%d", ((data[0] >> (7-i)) & 1));
    }
    printf("\n");

  }
  else {
    printf("Failed to initialize MPSSE: %s\n", ErrorString(ftdi));
  }

  // start read thread
  
  Can_read_thread_ = std::thread(
      [this, parsefunction]() { this->read_device_loop(parsefunction); });
  
}

void CommCanSPI::write_to_device(std::vector<uint8_t> msg) {
  Can_write_mutex_.lock();
  uint32_t can_id = static_cast<uint32_t>((msg[0] << 24) + (msg[1] << 16) + (msg[2] << 8) + msg[3]);
  // Extract the TX0SIDH and TX0SIDL values from the CAN frame ID
  uint8_t tx0sidh = static_cast<uint8_t>((can_id >> 21) & 0x07);
  uint8_t tx0sidl = static_cast<uint8_t>((can_id >> 18) & 0xFF);
  // Extract the TX0EID8 and TX0EID0 values from the CAN frame ID
  uint8_t tx0eid8 = static_cast<uint8_t>((can_id >> 8) & 0xFF);
  uint8_t tx0eid0 = static_cast<uint8_t>(can_id & 0xFF);
  
  std::cout << "Expected CAN message with ID: ";
  printf("0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",tx0sidh, tx0sidl, tx0eid8, tx0eid0, msg[4], msg[5], msg[6], msg[7], msg[8]);

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
      tx0sidh,
      tx0sidl,
      tx0eid8,
      tx0eid0,
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
    Write(ftdi, "\x81", 1);
    Stop(ftdi);

    Start(ftdi);
    Write(ftdi, "\x03\x30", 2);
    data = Read(ftdi, 1);
    Stop(ftdi);

    printf("TXB0 is now: 0x%02x | ", data[0]);
    for(int i = 0; i < 8; i++){
      printf("%d", ((data[0] >> (7-i)) & 1));
    }
    printf("\n");

    printf("Reading TEC...\n");
    Start(ftdi);
    Write(ftdi, "\x03\x1C", 2);
    data = Read(ftdi, 1);
    Stop(ftdi);

    printf("TEC is now: 0x%02x | ", data[0]);
    for(int i = 0; i < 8; i++){
      printf("%d", ((data[0] >> (7-i)) & 1));
    }
    printf("\n");
    
  }
  
  Can_write_mutex_.unlock();
}

void CommCanSPI::read_device_loop(std::function<void(std::vector<uint8_t>)> parsefunction) {
  std::chrono::milliseconds time_last =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());
  char* read_buffer;
  char read_cmd[2]{
    MCP_CMD_READ,
    0x61
  };
  while (true) {
    Can_write_mutex_.lock();
    Start(ftdi);
    Write(ftdi, read_cmd, sizeof(read_cmd));
    read_buffer = Read(ftdi, 14);
    Stop(ftdi);
    Can_write_mutex_.unlock();
    int num_bytes = sizeof(read_buffer);
    //int num_bytes = 0; // to implement read
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
      printf("Read Byte[%d/%d]: 0x%x\n", i + 1, num_bytes, read_buffer[i]);
      msg.push_back(read_buffer[i]);
    }
    
    try{
      parsefunction(msg);
      is_connected_ = true;
    } catch(int i){
      if (i == -4){
        printf("Not a valid RPM packet. Connection = False\n");
      }
    }
    msg.clear();
  }
}


bool CommCanSPI::is_connected() { return (is_connected_); }

}  // namespace RoverRobotics
