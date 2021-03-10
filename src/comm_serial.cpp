
#include "comm_serial.hpp"

namespace RoverRobotics {
CommSerial::CommSerial(const char *device,
                       std::function<void(std::vector<uint32_t>)> parsefunction,
                       std::vector<uint32_t> setting) {
  // open serial port at specified port
  serial_port = open(device, 02);

  struct termios tty;
  if (tcgetattr(serial_port, &tty) != 0) {
    throw(-1);
    return;
  }
  tty.c_cflag &= ~PARENB;  // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB;  // Clear stop field, only one stop bit used in
                           // communication (most common)
  tty.c_cflag &= ~CSIZE;   // Clear all bits that set the data size tty.c_cflag
                           // |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &=
      ~CRTSCTS;  // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |=
      CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;    // Disable echo
  tty.c_lflag &= ~ECHOE;   // Disable erasure
  tty.c_lflag &= ~ECHONL;  // Disable new-line echo
  tty.c_lflag &= ~ISIG;    // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                   ICRNL);  // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST;  // Prevent special interpretation of output bytes
                          // (e.g. newline chars)
  tty.c_oflag &=
      ~ONLCR;  // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VTIME] = 0;  // remove wait time

  // Set in/out baud rate
  cfsetispeed(&tty, (int)setting[0]);
  cfsetospeed(&tty, (int)setting[0]);
  read_size_ = (int)setting[1];
  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    throw(-1);
    return;
  }
  serial_read_thread = std::thread(
      [this, parsefunction]() { this->read_device_loop(parsefunction); });
}

void CommSerial::write_to_device(std::vector<uint32_t> msg) {
  serial_write_mutex.lock();
  if (serial_port >= 0) {
    unsigned char write_buffer[msg.size()];
    for (int x = 0; x < msg.size(); x++) {
      write_buffer[x] = msg[x];
    }

    write(serial_port, write_buffer, msg.size());
  }
  serial_write_mutex.unlock();
}

void CommSerial::read_device_loop(
    std::function<void(std::vector<uint32_t>)> parsefunction) {
  while (true) {
    unsigned char read_buf[read_size_];
    int num_bytes = read(serial_port, &read_buf, read_size_);
    if (num_bytes <= 0) {
      continue;
    }
    static std::vector<uint32_t> output;
    for (int x = 0; x < num_bytes; x++) {
      output.push_back(read_buf[x]);
    }
    parsefunction(output);
    output.clear();
  }
}

bool CommSerial::is_connected() { return (serial_port > 0); }

}  // namespace RoverRobotics