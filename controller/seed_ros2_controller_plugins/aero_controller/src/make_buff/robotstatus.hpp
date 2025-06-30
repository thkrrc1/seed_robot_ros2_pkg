#pragma once

#include <seed_ros2_controller/command_interface/other_command/other_cmd_buff.hpp>
#include <aero_controller_msgs/msg/robot_status.hpp>

namespace aero_controller {

bool makeBuffRobotStatus(const std::string &protocol, BuffRaw &buff, int msid);

bool getDataRobotStatus(const std::string &protocol, const BuffRaw &buff, aero_controller_msgs::msg::RobotStatus &status);

}
