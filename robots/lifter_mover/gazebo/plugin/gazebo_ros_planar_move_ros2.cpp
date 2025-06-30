#include "gazebo_ros_planar_move_ros2.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <sdf/Element.hh>

#include <chrono>
#include <memory>
#include <string>

using namespace gazebo_ros2;

PlanarMovePlugin::PlanarMovePlugin() = default;
PlanarMovePlugin::~PlanarMovePlugin() = default;

void PlanarMovePlugin::Configure(const gz::sim::Entity &entity,
                                 const std::shared_ptr<const sdf::Element> &sdf,
                                 gz::sim::EntityComponentManager &ecm,
                                 gz::sim::EventManager &)
{
  this->model_ = gz::sim::Model(entity);
  if (!this->model_.Valid(ecm))
  {
    std::cerr << "Invalid model entity." << std::endl;
    return;
  }

  if (sdf->HasElement("link_name"))
    this->link_name_ = sdf->Get<std::string>("link_name");
  else
    this->link_name_ = this->model_.Name(ecm);

  this->link_entity_ = this->model_.LinkByName(ecm, this->link_name_);
  if (this->link_entity_ == gz::sim::kNullEntity)
  {
    std::cerr << "Link [" << this->link_name_ << "] not found." << std::endl;
    return;
  }

  if (sdf->HasElement("use_force"))
    this->use_force_ = sdf->Get<bool>("use_force");

  int argc = 0;
  char **argv = nullptr;
  if (!rclcpp::ok())
    rclcpp::init(argc, argv);

  this->node_ = std::make_shared<rclcpp::Node>("planar_move_plugin");

  this->cmd_vel_sub_ = this->node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&PlanarMovePlugin::CmdVelCallback, this, std::placeholders::_1));

  this->odom_pub_ = this->node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->node_);

  this->last_time_ = this->node_->now();

  std::thread([this]() {
    rclcpp::spin(this->node_);
  }).detach();
}

void PlanarMovePlugin::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(this->cmd_vel_mutex_);
  this->cmd_vel_ = *msg;
}

void PlanarMovePlugin::PreUpdate(const gz::sim::UpdateInfo &info,
                                 gz::sim::EntityComponentManager &ecm)
{
  if (info.paused)
    return;

  std::lock_guard<std::mutex> lock(this->cmd_vel_mutex_);
  auto pose = gz::sim::worldPose(this->link_entity_, ecm);
  auto linear = gz::math::Vector3d(this->cmd_vel_.linear.x, this->cmd_vel_.linear.y, 0);
  auto angular = gz::math::Vector3d(0, 0, this->cmd_vel_.angular.z);

  gz::sim::Link link(this->link_entity_);
  link.SetLinearVelocity(ecm, linear);
  link.SetAngularVelocity(ecm, angular);

  auto now = this->node_->now();
  this->last_time_ = now;

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";
  odom.pose.pose.position.x = pose.Pos().X();
  odom.pose.pose.position.y = pose.Pos().Y();
  odom.pose.pose.orientation.z = pose.Rot().Z();
  odom.twist.twist.linear.x = linear.X();
  odom.twist.twist.linear.y = linear.Y();
  odom.twist.twist.angular.z = angular.Z();
  this->odom_pub_->publish(odom);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = now;
  tf.header.frame_id = "odom";
  tf.child_frame_id = "base_footprint";
  tf.transform.translation.x = pose.Pos().X();
  tf.transform.translation.y = pose.Pos().Y();
  tf.transform.translation.z = pose.Pos().Z();
  tf.transform.rotation.x = pose.Rot().X();
  tf.transform.rotation.y = pose.Rot().Y();
  tf.transform.rotation.z = pose.Rot().Z();
  tf.transform.rotation.w = pose.Rot().W();
  this->tf_broadcaster_->sendTransform(tf);
}

GZ_ADD_PLUGIN(
    gazebo_ros2::PlanarMovePlugin,
    gz::sim::System,
    gazebo_ros2::PlanarMovePlugin::ISystemConfigure,
    gazebo_ros2::PlanarMovePlugin::ISystemPreUpdate)
