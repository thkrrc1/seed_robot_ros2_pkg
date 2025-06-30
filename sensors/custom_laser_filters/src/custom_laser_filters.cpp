#include <sensor_msgs/msg/laser_scan.hpp>
#include <filters/filter_base.hpp>

#include "custom_laser_filters/edge_effect_filter.hpp"

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(custom_laser_filters::EdgeEffectFilter, filters::FilterBase<sensor_msgs::msg::LaserScan>)
