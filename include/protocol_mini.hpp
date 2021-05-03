#pragma once
#include "protocol_pro_2.hpp"

namespace RoverRobotics {
class MiniProtocolObject;
}

class RoverRobotics::MiniProtocolObject
    : public RoverRobotics::Pro2ProtocolObject {
 private:
  /* metric units (meters) */
  Control::robot_geometry robot_geometry_ = {
      .intra_axle_distance = 0.205,
      .wheel_base = 0.265,
      .wheel_radius = 0.09,
      .center_of_mass_x_offset = 0,
      .center_of_mass_y_offset = 0};
};