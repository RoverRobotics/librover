#pragma once
#include "comm_base.hpp"

namespace RoverRobotics {
class CommSerial;
}
class RoverRobotics::CommSerial : public RoverRobotics::CommBase {
 public:
  CommSerial(const char *device, std::function<void(std::vector<uint32_t>)>, std::vector<uint32_t>);
  ~CommSerial();
  void write_to_device(std::vector<uint32_t> msg);
  void read_from_device(std::function<void(std::vector<uint32_t>)>);

  bool is_connected();

 private:
  std::mutex writemutex;
  std::mutex readmutex;
  int write_size_;
  int read_size_;
  int serial_port;
  std::thread readthread;
};
