#pragma once
#include <cstdint>
#include <optional>
#include <vector>

namespace vesc {
class BridgedVescArray;

typedef struct {
  int vescId;
  float current;
  float rpm;
  float duty;
  bool dataValid;
} vescChannelStatus;

enum vescPacketFlags : uint32_t {
  PACKET_FLAG = 0x80000000,
  RPM = 0x00000900,
  CURRENT = 0x00000100,
  DUTY = 0x00000000
};

typedef struct {
  uint8_t vescId;
  vescPacketFlags commandType;
  int32_t commandValue;
} vescChannelCommand;

/*
scaling factors:
multiply values FROM VESC after receiving
divide values TO VESC before sending
*/

const uint32_t RPM_SCALING_FACTOR = 60 / 1000;
const uint32_t DUTY_SCALING_FACTOR = 1 / 10;
const uint32_t CURRENT_SCALING_FACTOR = 1 / 10;
const uint32_t DUTY_COMMAND_SCALING_FACTOR = 100000;

const uint32_t CONTENT_MASK = 0xFFFFFF00;
const uint32_t ID_MASK = 0x000000FF;
const uint32_t SEND_MSG_LENGTH = 4;

}  // namespace vesc

class vesc::BridgedVescArray {
 public:
  BridgedVescArray(std::vector<uint8_t> vescIds = std::vector<uint8_t>{0, 1, 2, 3});
  vesc::vescChannelStatus parseReceivedMessage(
      std::vector<uint32_t> robotmsg);
  std::vector<uint32_t> buildCommandMessage(
      vesc::vescChannelCommand command);

 private:
  std::vector<uint8_t> vescIds_;
};
