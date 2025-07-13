#pragma once

#include <iostream>
#include <cstdio>

#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("ms_stub"),__VA_ARGS__)
#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("ms_stub"),__VA_ARGS__)
#define ROS_INFO_STREAM(...) RCLCPP_INFO_STREAM(rclcpp::get_logger("ms_stub"), __VA_ARGS__)
#define ROS_ERROR_STREAM(...) RCLCPP_ERROR_STREAM(rclcpp::get_logger("ms_stub"),  __VA_ARGS__)
