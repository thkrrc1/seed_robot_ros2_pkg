#pragma once

#include <seed_ros2_controller/command_interface/other_command/other_cmd_buff.hpp>

#include <seed_ros2_controller/aero/version3/send_buff_aero3.hpp>
#include <seed_ros2_controller/aero/version4/send_buff_aero4.hpp>

namespace aero_controller {

bool makeBuffGetVersion(const std::string &protocol, BuffRaw &buff, uint8_t msid, uint8_t mcid);

}
