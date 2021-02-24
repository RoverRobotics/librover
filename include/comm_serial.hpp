#pragma once
#include "comm_base.hpp"

namespace RoverRobotics {
class CommSerial;
}
class RoverRobotics::CommSerial : public RoverRobotics::CommBase {
 public:
  /*
   * @brief Constructor For Serial Communication
   * This contructor accepts a serial device path, a callback function for
   * reading from the serial device and a std::vector<uint32_t> for settings.
   * Callback funcion is automatically started as a thread inside this
   * contructor
   *
   * @param device the device path
   * @param callbackfunction
   * @param settings
   */
  CommSerial(const char *device, std::function<void(std::vector<uint32_t>)>,
             std::vector<uint32_t>);
  /*
   * @brief Write data to Serial Device
   * by accepting a vector of unsigned int 32 and convert it to a byte stream
   * @param msg message to convert and write to device
   */
  void write_to_device(std::vector<uint32_t> msg);
  /*
   * @brief Read data from Serial Device
   * by reading the current device buffer then convert to a vector of unsigned
   * int 32.
   * @param callback to process the unsigned int 32.
   */
  void read_from_device(std::function<void(std::vector<uint32_t>)>);
  /*
   * @brief Check if Serial device is still connected by check the state of the
   * file descriptor
   * @return bool file descriptor state
   */
  bool is_connected();

 private:
  std::mutex serial_write_mutex;
  int read_size_;
  int serial_port;
  std::thread serial_read_thread;
};
