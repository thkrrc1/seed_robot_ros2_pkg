#ifndef GAZEBO_ROS_PLANAR_MOVE_ROS2_HPP
#define GAZEBO_ROS_PLANAR_MOVE_ROS2_HPP

#include <memory>
#include <string>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>

namespace gazebo_ros2
{

class PlanarMovePlugin : public gz::sim::System,
                         public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPreUpdate
{
public:
  PlanarMovePlugin();
  ~PlanarMovePlugin() override;

  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override;

private:
  void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  gz::sim::Model model_;
  gz::sim::Entity link_entity_;
  std::string link_name_;
  bool use_force_{false};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  gz::math::Pose3d last_pose_;
  gz::math::Vector3d last_linear_vel_;
  gz::math::Vector3d last_angular_vel_;
  rclcpp::Time last_time_;

  geometry_msgs::msg::Twist cmd_vel_;
  std::mutex cmd_vel_mutex_;
};

}  // namespace gazebo_ros2

#endif  // GAZEBO_ROS_PLANAR_MOVE_ROS2_HPP