#include <rclcpp/rclcpp.hpp>
#include "motion_player.hpp"

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(motion_player::MotionPlayer, controller_interface::ControllerInterface)
