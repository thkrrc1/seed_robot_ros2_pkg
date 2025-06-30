#pragma once

#include <seed_ros2_controller/command_interface/other_command/other_cmd_buff.hpp>
#include <aero_controller_msgs/msg/status.hpp>

namespace aero_controller {

bool makeBuff(const std::string &protocol, BuffRaw &buff, int msid);

bool getData(const std::string &protocol, const BuffRaw &buff, aero_controller_msgs::msg::Status &status);

}
