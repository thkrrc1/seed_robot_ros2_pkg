#ifndef ASSISTED_TELEOP__ASSISTED_TELEOP_HPP_
#define ASSISTED_TELEOP__ASSISTED_TELEOP_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

#include "std_msgs/msg/bool.hpp"

namespace assisted_teleop
{

class AssistedTeleop : public rclcpp::Node
{
public:
  AssistedTeleop();
  ~AssistedTeleop() override;

private:
  void velCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void footprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
  void onAssistedTeleopCallback(const std_msgs::msg::Bool & msg);
  void controlLoop();

  bool getFeasibleVel(const Eigen::Vector3f & desired_vel, Eigen::Vector3f & feasible_vel);
  bool getMinimumVel(const Eigen::Vector3f & desired_vel, Eigen::Vector3f & tgt_vel) const;
  bool checkTrajectory(const Eigen::Vector3f & vel);
  double footprintCostAtPose(double x, double y, double yaw);

  void updateCostmapFromMsg(const nav_msgs::msg::OccupancyGrid & msg);
  void setDefaultFootprint();
  unsigned char occupancyToCost(int8_t occupancy) const;
  geometry_msgs::msg::Twist toTwist(const Eigen::Vector3f & vel) const;
  Eigen::Vector3f readDesiredVelAndUpdateTimeout();

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr on_assisted_teleop_sub_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  mutable std::mutex mtx_;

  nav2_costmap_2d::Costmap2D costmap_;
  nav2_costmap_2d::Footprint footprint_;

  Eigen::Vector3f desired_vel_;
  rclcpp::Time last_received_;

  bool has_request_ = false;
  bool has_costmap_ = false;
  bool has_footprint_ = false;

  int num_x_samples_ = 20;
  int num_th_samples_ = 10;
  double theta_range_ = 0.7;

  double collision_trans_speed_ = 0.0;
  double collision_rot_speed_ = 0.0;

  double controller_frequency_ = 100.0;
  double cmd_vel_timeout_ = 0.5;

  double sim_time_ = 1.5;
  double sim_granularity_ = 0.05;

  double footprint_length_ = 0.7;
  double footprint_width_ = 0.5;

  double slow_cost_ = 150.0;
  double stop_cost_ = static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
  double min_scale_ = 0.2;
  bool use_cost_scaling_ = true;
  bool fail_open_without_costmap_ = false;

  std::string input_cmd_vel_topic_ = "/mechanum_controller/cmd_vel_teleop_raw";
  std::string output_cmd_vel_topic_ = "/mechanum_controller/cmd_vel_teleop";
  std::string costmap_topic_ = "/local_costmap/costmap";
  std::string footprint_topic_ = "/local_costmap/published_footprint";
  std::string notify_on_assisted_teleop_topic_ = "/on_assisted_teleop";

  std::string base_frame_ = "base_link";
  std::string costmap_frame_ = "odom";

  std::atomic<bool> on_assited_teleop_ = false;
};

}  // namespace assisted_teleop

#endif  // ASSISTED_TELEOP__ASSISTED_TELEOP_HPP_
