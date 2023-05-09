#include "comm_can_spi.hpp"
namespace RoverRobotics {


/*
HELPER FUNCTIONS - Only used in this class
*/

static void _print_bits(char* data){
  for(int i = 0; i < 8; i++){
    printf("%d", ((data[0] >> (7-i)) & 1));
  }
  printf("\n");
}

static void _clear_can_int(mpsse_context* ftdi){
  Start(ftdi);
  Write(ftdi, "\x02\x2C\x00", 3);
  Stop(ftdi);
}

static char* _get_rec(mpsse_context* ftdi){
  Start(ftdi);
  Write(ftdi, "\x03\x1D", 2);
  char* data = Read(ftdi, 1);
  Stop(ftdi);
  return data;
}

static char* _get_tec(mpsse_context* ftdi){
  Start(ftdi);
  Write(ftdi, "\x03\x1D", 2);
  char* data = Read(ftdi, 1);
  Stop(ftdi);
  return data;
}

static char* _get_eflg(mpsse_context* ftdi){
  Start(ftdi);
  Write(ftdi, "\x03\x2D", 2);
  char* data = Read(ftdi, 1);
  Stop(ftdi);
  return data;
}

static char* _get_canintf(mpsse_context* ftdi){
  Start(ftdi);
  Write(ftdi, "\x03\x2D", 2);
  char* data = Read(ftdi, 1);
  Stop(ftdi);
  return data;
}

static void _transmit_message(mpsse_context* ftdi){
  Start(ftdi);
  Write(ftdi, "\x81", 1);
  Stop(ftdi);
}

// Configures MPSSE | returns 1 on success & returns 0 on failure
static int _configure_mpsse_context(mpsse_context* ftdi){
  
}

void spin_one_wheel(mpsse_context* ftdi){
  char send_one_msg[] = {
    MCP_CMD_WRITE,
    0x31,
    0x00,
    0x08,
    0x01,
    0x01,
    0x04,
    0x00,
    0x00,
    0x01,
    0x68
  };
  printf("Sending 360 erpm...\n");
  Start(ftdi);
  Write(ftdi, send_one_msg, sizeof(send_one_msg));
  Stop(ftdi);

  _transmit_message(ftdi);

  sleep(0.05);

  _clear_can_int(ftdi);
}

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
    0x84,
    0xF6,
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
    0x00,
    0x08,
    0x01,
    0x01,
    0x04,
    0x00,
    0x00,
    0x00,
    0x00
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

    // Check data 
    printf("CANCTRL Data Received: 0x%02x | ", data[0]);
    _print_bits(data);

    Start(ftdi);
    Write(ftdi, read_canstat_cmd, sizeof(read_canstat_cmd));
    data = Read(ftdi, 1);
    Stop(ftdi);

    printf("CANSTAT Data Received: 0x%02x | ", data[0]);
    for(int i = 0; i < 8; i++){
      printf("%d", ((data[0] >> (7-i)) & 1));
    }
    printf("\n");


    // Configure CANCTRl and baud rate settings
    printf("Configuring CANCTRL and CNF[3:1]\n");
    Start(ftdi);
    Write(ftdi, conf_canctrl, sizeof(conf_canctrl));
    Stop(ftdi);

    Start(ftdi);
    Write(ftdi, conf_can_cmd, sizeof(conf_can_cmd));
    Stop(ftdi);


    // Clear CANINTF
    _clear_can_int(ftdi);

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
    _print_bits(data);

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
    for (int i = 0; i < 9; i++) {
      printf("0x%02x ", data[i]);
    }
    std::cout << std::endl;

    Start(ftdi);
    Write(ftdi, conf_no_int, sizeof(conf_no_int));
    Stop(ftdi);

    sleep(0.100);

    Start(ftdi);
    Write(ftdi, "\x81", 2);
    Stop(ftdi);
    
    sleep(0.05);

    Start(ftdi);
    Write(ftdi, "\x03\x30", 2);
    data = Read(ftdi, 1);
    Stop(ftdi);

    printf("TXB0 is now: 0x%02x | ", data[0]);
    _print_bits(data);

    sleep(0.05);

    printf("Reading TEC...\n");
    data = _get_tec(ftdi);

    printf("TEC is now: 0x%02x | ", data[0]);
    _print_bits(data);

    printf("Reading REC...\n");
    data = _get_rec(ftdi);

    printf("REC is now: 0x%02x | ", data[0]);
    _print_bits(data);

    printf("Reading CANINTF...\n");
    data = _get_canintf(ftdi);

    printf("CANINTF is now: 0x%02x | ", data[0]);
    _print_bits(data);

    printf("Reading EFLG...\n");
    data = _get_eflg(ftdi);

    printf("EFLG is now: 0x%02x | ", data[0]);
    _print_bits(data);
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

  uint8_t tbufdata[4];

  uint16_t canid = (uint16_t)(can_id & 0x0FFFF);

  tbufdata[3] = (uint8_t) (canid & 0xFF);
  tbufdata[2] = (uint8_t) (canid >> 8);
  canid = (uint16_t)(can_id >> 16);
  tbufdata[1] = (uint8_t) (canid & 0x03);
  tbufdata[1] += (uint8_t) ((canid & 0x1C) << 3);
  tbufdata[1] |= 0x08;
  tbufdata[0] = (uint8_t) (canid >> 5 );
  // Extract the TX0SIDH and TX0SIDL values from the CAN frame ID
  uint8_t tx0sidh = tbufdata[0];
  uint8_t tx0sidl = tbufdata[1];
  // Extract the TX0EID8 and TX0EID0 values from the CAN frame ID
  uint8_t tx0eid8 = tbufdata[2];
  uint8_t tx0eid0 = tbufdata[3];
  
  std::cout << "Expected CAN message with ID: ";
  printf("0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",tx0sidh, tx0sidl, tx0eid8, tx0eid0, msg[4], msg[5], msg[6], msg[7], msg[8]);

  if (msg.size() == CAN_MSG_SIZE_) {
    // convert msg to spi frame
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
    
    _transmit_message(ftdi);

    Start(ftdi);
    Write(ftdi, "\x03\x30", 2);
    data = Read(ftdi, 1);
    Stop(ftdi);

    printf("TXB0 is now: 0x%02x | ", data[0]);
    _print_bits(data);

    printf("Reading TEC...\n");
    data = _get_tec(ftdi);

    printf("TEC is now: 0x%02x | ", data[0]);
    _print_bits(data);
  }  
  Can_write_mutex_.unlock();
}

void CommCanSPI::read_device_loop(std::function<void(std::vector<uint8_t>)> parsefunction) {
  std::chrono::milliseconds time_last =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());
  char* read_buffer_id;
  char* read_buffer_dlc;
  char* read_dlc_size;
  char* data;

  while (true) {
    Can_write_mutex_.lock();
    Start(ftdi);
    Write(ftdi, "\x03\x61", 2);
    read_buffer_id = Read(ftdi, 4);
    Stop(ftdi);

    Start(ftdi);
    Write(ftdi, "\x03\x65", 2);
    read_dlc_size = Read(ftdi, 1);
    Stop(ftdi);

    int dlc_size = (short) strtol(read_dlc_size, NULL, 16);

    Start(ftdi);
    Write(ftdi, "\x03\x66", 2);
    read_buffer_dlc = Read(ftdi, dlc_size);
    Stop(ftdi);
    
    _clear_can_int(ftdi);

    printf("Reading CANINTF...\n");
    data = _get_canintf(ftdi);

    Can_write_mutex_.unlock();

    printf("CANINTF is now: 0x%02x | ", data[0]);
    _print_bits(data);

    uint32_t can_id = (read_buffer_id[0] << 3) + (read_buffer_id[1] << 5);
    can_id = (can_id << 2) + (read_buffer_id[1] & 0x03);
    can_id = (can_id << 8) + (read_buffer_id[2]);
    can_id = (can_id << 8) + (read_buffer_id[3]);
    printf("Read CAN ID: 0x%x\n", can_id);
    int num_bytes = dlc_size;

    is_connected_ = true;

    std::vector<uint8_t> msg;
    msg.push_back((static_cast<uint8_t>((can_id >> 24) & 0xFF)));
    msg.push_back((static_cast<uint8_t>((can_id >> 16) & 0xFF)));
    msg.push_back((static_cast<uint8_t>((can_id >> 8) & 0xFF)));
    msg.push_back((static_cast<uint8_t>(can_id & 0xFF)));
    msg.push_back(read_dlc_size[0]);

    for (int i = 0; i < dlc_size; i++) {
      printf("Read Byte[%d/%d]: 0x%02x\n", i + 1, num_bytes, read_buffer_dlc[i]);
      msg.push_back(read_buffer_dlc[i]);
    }
    
    try{
      parsefunction(msg);
      is_connected_ = true;
    } catch(int i){
      if (i == -4){
        printf("Failed to pass content mask.\n");
      }
    }
    msg.clear();
  }
}



bool CommCanSPI::is_connected() { return (is_connected_); }

}  // namespace RoverRobotics
