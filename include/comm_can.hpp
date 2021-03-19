#pragma once
#include "comm_base.hpp"

namespace RoverRobotics {
class CommCan;
}
class RoverRobotics::CommCan : public RoverRobotics::CommBase {
 public:
  /*
   * @brief Constructor For Can Communication
   * This contructor accepts a Can device path, a callback function for
   * reading from the Can device and a std::vector<uint32_t> for settings.
   * Callback funcion is automatically started as a thread inside this
   * contructor
   *
   * @param device the device path
   * @param callbackfunction
   * @param settings
   */
  CommCan(const char *device, std::function<void(std::vector<uint32_t>)>,
             std::vector<uint32_t>);
  /*
   * @brief Write data to Can Device
   * by accepting a vector of unsigned int 32 and convert it to a byte stream
   * @param msg message to convert and write to device
   */
  void write_to_device(std::vector<uint32_t> msg);
  /*
   * @brief Read data from Can Device
   * by reading the current device buffer then convert to a vector of unsigned
   * int 32.
   * @param callback to process the unsigned int 32.
   */
  void read_device_loop(std::function<void(std::vector<uint32_t>)>);
  /*
   * @brief Check if Can device is still connected by check the state of the
   * file descriptor
   * @return bool file descriptor state
   */
  bool is_connected();

 private:
  std::mutex Can_write_mutex_;
  int read_size_;
  int Can_port_;
  std::atomic<bool> is_connected_;
  std::thread Can_read_thread_;
  const int TIMEOUT_MS_ = 1000; //1 sec timeout
};
