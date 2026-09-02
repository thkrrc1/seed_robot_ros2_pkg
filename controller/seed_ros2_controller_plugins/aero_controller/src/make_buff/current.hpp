#pragma once

#include <seed_ros2_controller/command_interface/other_command/other_cmd_buff.hpp>

#include <aero_controller_msgs/msg/current.hpp>

namespace aero_controller {

bool makeBuffCurrent(const std::string &protocol, BuffRaw &buff, int msid);

bool getDataCurrent(const std::string &protocol, const BuffRaw &buff, aero_controller_msgs::msg::Current &status);

}
