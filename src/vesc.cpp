#include "vesc.hpp"

namespace vesc {

BridgedVescArray::BridgedVescArray(std::vector<uint8_t> vescIds) {
  vescIds_ = vescIds;
}

std::optional<vescChannelStatus> BridgedVescArray::parseReceivedMessage(
    std::vector<uint32_t> robotmsg) {
  /* process valid RPM packets */
  if ((robotmsg[0] & CONTENT_MASK) ==
      (vescPacketFlags::PACKET_FLAG | vescPacketFlags::RPM)) {
    uint8_t vescId = robotmsg[0] & ID_MASK;

    /* combine shifted byte values into a single rpm value */
    int32_t rpm_scaled = ((uint8_t)robotmsg[2] << 24) |
                         ((uint8_t)robotmsg[3] << 16) |
                         ((uint8_t)robotmsg[4] << 8) | ((uint8_t)robotmsg[5]);

    /* combine shifted byte values into a single current value */
    int16_t current_scaled =
        ((uint8_t)robotmsg[6] << 8) | ((uint8_t)robotmsg[7]);

    /* combine shifted byte values into a single duty value */
    int16_t duty_scaled =
        (((uint8_t)robotmsg[8] << 8) | ((uint8_t)robotmsg[9]));

    /* scale values per fixed-point vesc protocol */
    float rpm = (float)rpm_scaled * RPM_SCALING_FACTOR;
    float current = (float)current_scaled * CURRENT_SCALING_FACTOR;
    float duty = (float)duty_scaled * DUTY_SCALING_FACTOR;

    return (vescChannelStatus){
        .vescId = vescId, .current = current, .rpm = rpm, .duty = duty};
  } else {
    return {};
  }
}

std::vector<uint32_t> BridgedVescArray::buildCommandMessage(
    vesc::vescChannelCommand command) {
  /* create a vector to hold the message */
  std::vector<uint32_t> write_buffer;

  /* build the message */
  switch (command.commandType) {
    case (RPM):
      command.commandValue /= RPM_SCALING_FACTOR;
      break;
    case (CURRENT):
      command.commandValue /= CURRENT_SCALING_FACTOR;
      break;
    default:
      exit(-1);
  };

  write_buffer = {
      command.vescId | vescPacketFlags::PACKET_FLAG | command.commandType,
      SEND_MSG_LENGTH,
      static_cast<uint8_t>((static_cast<uint32_t>(command.commandValue) >> 24) &
                           0xFF),
      static_cast<uint8_t>((static_cast<uint32_t>(command.commandValue) >> 16) &
                           0xFF),
      static_cast<uint8_t>((static_cast<uint32_t>(command.commandValue) >> 8) &
                           0xFF),
      static_cast<uint8_t>(static_cast<uint32_t>(command.commandValue) & 0xFF)};

  return write_buffer;
}

}  // namespace vesc