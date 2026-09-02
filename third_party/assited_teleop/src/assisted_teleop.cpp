#include "assisted_teleop.hpp"

#include <chrono>
#include <functional>

namespace assisted_teleop
{

AssistedTeleop::AssistedTeleop()
: rclcpp::Node("assisted_teleop"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  desired_vel_(Eigen::Vector3f::Zero()),
  last_received_(this->now())
{
  this->declare_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->declare_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->declare_parameter("costmap_topic", costmap_topic_);
  this->declare_parameter("footprint_topic", footprint_topic_);
  this->declare_parameter("base_frame", base_frame_);
  this->declare_parameter("costmap_frame", costmap_frame_);

  this->declare_parameter("controller_frequency", controller_frequency_);
  this->declare_parameter("cmd_vel_timeout", cmd_vel_timeout_);

  this->declare_parameter("num_x_samples", num_x_samples_);
  this->declare_parameter("num_th_samples", num_th_samples_);
  this->declare_parameter("theta_range", theta_range_);
  this->declare_parameter("translational_collision_speed", collision_trans_speed_);
  this->declare_parameter("rotational_collision_speed", collision_rot_speed_);

  this->declare_parameter("sim_time", sim_time_);
  this->declare_parameter("sim_granularity", sim_granularity_);

  this->declare_parameter("footprint_length", footprint_length_);
  this->declare_parameter("footprint_width", footprint_width_);

  this->declare_parameter("slow_cost", slow_cost_);
  this->declare_parameter("stop_cost", stop_cost_);
  this->declare_parameter("min_scale", min_scale_);
  this->declare_parameter("use_cost_scaling", use_cost_scaling_);
  this->declare_parameter("fail_open_without_costmap", fail_open_without_costmap_);

  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("costmap_topic", costmap_topic_);
  this->get_parameter("footprint_topic", footprint_topic_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("costmap_frame", costmap_frame_);

  this->get_parameter("controller_frequency", controller_frequency_);
  this->get_parameter("cmd_vel_timeout", cmd_vel_timeout_);

  this->get_parameter("num_x_samples", num_x_samples_);
  this->get_parameter("num_th_samples", num_th_samples_);
  this->get_parameter("theta_range", theta_range_);
  this->get_parameter("translational_collision_speed", collision_trans_speed_);
  this->get_parameter("rotational_collision_speed", collision_rot_speed_);

  this->get_parameter("sim_time", sim_time_);
  this->get_parameter("sim_granularity", sim_granularity_);

  this->get_parameter("footprint_length", footprint_length_);
  this->get_parameter("footprint_width", footprint_width_);

  this->get_parameter("slow_cost", slow_cost_);
  this->get_parameter("stop_cost", stop_cost_);
  this->get_parameter("min_scale", min_scale_);
  this->get_parameter("use_cost_scaling", use_cost_scaling_);
  this->get_parameter("fail_open_without_costmap", fail_open_without_costmap_);

  setDefaultFootprint();

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    output_cmd_vel_topic_, rclcpp::QoS(1));

  cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, rclcpp::QoS(10),
    std::bind(&AssistedTeleop::velCallback, this, std::placeholders::_1));

  auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic_, costmap_qos,
    std::bind(&AssistedTeleop::costmapCallback, this, std::placeholders::_1));

  footprint_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
    footprint_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&AssistedTeleop::footprintCallback, this, std::placeholders::_1));

  auto notify_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  on_assisted_teleop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    notify_on_assisted_teleop_topic_, notify_qos,
    std::bind(&AssistedTeleop::onAssistedTeleopCallback, this, std::placeholders::_1));

  if (controller_frequency_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "controller_frequency must be positive. Use 10.0 Hz.");
    controller_frequency_ = 10.0;
  }

  const auto period = std::chrono::duration<double>(1.0 / controller_frequency_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&AssistedTeleop::controlLoop, this));

  RCLCPP_INFO(
    this->get_logger(),
    "AssistedTeleop started. input=%s output=%s costmap=%s footprint=%s",
    input_cmd_vel_topic_.c_str(), output_cmd_vel_topic_.c_str(),
    costmap_topic_.c_str(), footprint_topic_.c_str());
}

AssistedTeleop::~AssistedTeleop() = default;

void AssistedTeleop::velCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);

  desired_vel_[0] = static_cast<float>(msg->linear.x);
  desired_vel_[1] = static_cast<float>(msg->linear.y);
  desired_vel_[2] = static_cast<float>(msg->angular.z);

  last_received_ = this->now();
  has_request_ = true;
}

void AssistedTeleop::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  updateCostmapFromMsg(*msg);
  has_costmap_ = true;
}

void AssistedTeleop::footprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);

  if (msg->polygon.points.empty()) {
    RCLCPP_WARN(this->get_logger(), "received empty footprint. Keep previous footprint.");
    return;
  }

  footprint_.clear();
  footprint_.reserve(msg->polygon.points.size());

  for (const auto & p32 : msg->polygon.points) {
    geometry_msgs::msg::Point p;
    p.x = p32.x;
    p.y = p32.y;
    p.z = p32.z;
    footprint_.push_back(p);
  }

  has_footprint_ = true;
}

void AssistedTeleop::onAssistedTeleopCallback(const std_msgs::msg::Bool & msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  on_assited_teleop_.store(msg.data);
}

void AssistedTeleop::controlLoop()
{
  Eigen::Vector3f desired_vel_tmp = readDesiredVelAndUpdateTimeout();

  if (!has_request_ && desired_vel_tmp.isZero(0.0f)) {
    return;
  }

  if (!on_assited_teleop_.load()) {
    cmd_pub_->publish(toTwist(desired_vel_tmp));
    return;
  }

  Eigen::Vector3f tgt_vel = Eigen::Vector3f::Zero();
  const bool found = getFeasibleVel(desired_vel_tmp, tgt_vel);

  if (!found) {
    getMinimumVel(desired_vel_tmp, tgt_vel);
  }

  cmd_pub_->publish(toTwist(tgt_vel));
}

Eigen::Vector3f AssistedTeleop::readDesiredVelAndUpdateTimeout()
{
  std::lock_guard<std::mutex> lock(mtx_);

  if (!has_request_) {
    return Eigen::Vector3f::Zero();
  }

  Eigen::Vector3f vel = desired_vel_;
  const double dt = (this->now() - last_received_).seconds();

  if (dt > cmd_vel_timeout_) {
    desired_vel_ = Eigen::Vector3f::Zero();
    has_request_ = false;
    return Eigen::Vector3f::Zero();
  }

  return vel;
}

bool AssistedTeleop::getFeasibleVel(
  const Eigen::Vector3f & desired_vel,
  Eigen::Vector3f & feasible_vel)
{
  if (checkTrajectory(desired_vel)) {
    feasible_vel = desired_vel;
    return true;
  }

  Eigen::Vector3f best = Eigen::Vector3f::Zero();
  double best_diff_line = std::numeric_limits<double>::max();
  double best_diff_rot = std::numeric_limits<double>::max();
  bool trajectory_found = false;

  const int scale_samples = std::max(1, num_x_samples_);
  const int th_samples = std::max(1, num_th_samples_);

  const double minimum_scale = std::clamp(min_scale_, 0.01, 1.0);

  const double d_scale = 1.0 / static_cast<double>(scale_samples);
  const double dth = theta_range_ / static_cast<double>(th_samples);
  const double start_th = desired_vel[2] - theta_range_ / 2.0;

  const bool translation_requested = std::hypot(desired_vel[0], desired_vel[1]) > 1.0e-6;
  const bool rotation_requested = std::abs(desired_vel[2]) > 1.0e-6;

  for (int i = 0; i <= scale_samples; ++i) {
    const double scale = std::max(minimum_scale, 1.0 - i * d_scale);

    Eigen::Vector3f check_vel = Eigen::Vector3f::Zero();
    check_vel[0] = static_cast<float>(desired_vel[0] * scale);
    check_vel[1] = static_cast<float>(desired_vel[1] * scale);

    for (int j = 0; j < th_samples; ++j) {
      check_vel[2] = static_cast<float>(start_th + j * dth);
      if (!desired_vel.isZero(1.0e-6f) && check_vel.isZero(1.0e-6f))
      {
        continue;
      }
      const Eigen::Vector3f diffs = desired_vel - check_vel;
      const double diff_line = diffs[0] * diffs[0] + diffs[1] * diffs[1];
      const double diff_rot = diffs[2] * diffs[2];

      if (!(diff_line < best_diff_line ||
          (std::abs(best_diff_line - diff_line) < 0.001 && diff_rot < best_diff_rot))) {
        continue;
      }

      if (checkTrajectory(check_vel)) {
        bool ok = true;

        if (collision_trans_speed_ > 0.0 || collision_rot_speed_ > 0.0) {
          const double trans_vel = std::hypot(check_vel[0], check_vel[1]);
          const double rot_vel = std::abs(check_vel[2]);

          if (translation_requested && collision_trans_speed_ > 0.0 && trans_vel < collision_trans_speed_) {
            ok = false;
          }

          if (rotation_requested && collision_rot_speed_ > 0.0 && rot_vel < collision_rot_speed_)
          {
            ok = false;
          }
        }

        if (ok) {
          best = check_vel;
          best_diff_line = diff_line;
          best_diff_rot = diff_rot;
          trajectory_found = true;
        }
      }
    }

    if (best_diff_line != std::numeric_limits<double>::max()) {
      break;
    }
  }

  feasible_vel = best;
  return trajectory_found;
}

bool AssistedTeleop::getMinimumVel(const Eigen::Vector3f & desired_vel, Eigen::Vector3f & tgt_vel) const
{
  // 操作者が停止を指令した場合だけゼロを出力する
  if (desired_vel.isZero(1.0e-6f)) {
    tgt_vel = Eigen::Vector3f::Zero();
    return true;
  }

  const double trans_vel = std::hypot(desired_vel[0], desired_vel[1]);
  const double rot_vel = std::abs(desired_vel[2]);

  double scaling_factor = 1.0;
  bool collision_speed_configured = false;

  // 並進速度を衝突時速度まで下げる
  if (trans_vel > 1.0e-6 && collision_trans_speed_ > 0.0)
  {
    scaling_factor = std::min(scaling_factor, std::min(1.0, collision_trans_speed_ / trans_vel));
    collision_speed_configured = true;
  }

  // 旋回速度を衝突時速度まで下げる
  if (rot_vel > 1.0e-6 && collision_rot_speed_ > 0.0)
  {
    scaling_factor = std::min(scaling_factor, std::min(1.0, collision_rot_speed_ / rot_vel));
    collision_speed_configured = true;
  }

  // 対応するcollision speedが設定されていない場合でも、
  // min_scale_を使って停止を避ける
  if (!collision_speed_configured) 
  {
    scaling_factor = std::clamp(min_scale_, 0.01, 1.0);
  }

  // 操作者の指令方向を維持したまま低速化
  tgt_vel = static_cast<float>(scaling_factor) * desired_vel;
  return true;
}

bool AssistedTeleop::checkTrajectory(const Eigen::Vector3f & vel)
{
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if ((!has_costmap_ || !has_footprint_) && fail_open_without_costmap_) {
      return true;
    }
    if (!has_costmap_ || !has_footprint_) {
      return false;
    }
  }

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_.lookupTransform(costmap_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "TF lookup failed: %s", ex.what());
    return fail_open_without_costmap_;
  }

  double x = tf.transform.translation.x;
  double y = tf.transform.translation.y;
  double yaw = tf2::getYaw(tf.transform.rotation);

  const double vx = vel[0];
  const double vy = vel[1];
  const double wz = vel[2];

  double max_cost = 0.0;

  const double dt = std::max(0.01, sim_granularity_);
  for (double t = 0.0; t <= sim_time_; t += dt) {
    (void)t;

    const double cost = footprintCostAtPose(x, y, yaw);
    if (cost < 0.0 || cost >= stop_cost_) {
      return false;
    }
    max_cost = std::max(max_cost, cost);

    x += (vx * std::cos(yaw) - vy * std::sin(yaw)) * dt;
    y += (vx * std::sin(yaw) + vy * std::cos(yaw)) * dt;
    yaw += wz * dt;
  }

  if (use_cost_scaling_ && max_cost > slow_cost_) {
    return false;
  }

  return true;
}

double AssistedTeleop::footprintCostAtPose(double x, double y, double yaw)
{
  std::lock_guard<std::mutex> lock(mtx_);

  if (!has_costmap_ || !has_footprint_) {
    return -1.0;
  }

  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(&costmap_);
  return checker.footprintCostAtPose(x, y, yaw, footprint_);
}

void AssistedTeleop::updateCostmapFromMsg(const nav_msgs::msg::OccupancyGrid & msg)
{
  costmap_.resizeMap(
    msg.info.width,
    msg.info.height,
    msg.info.resolution,
    msg.info.origin.position.x,
    msg.info.origin.position.y);

  for (unsigned int y = 0; y < msg.info.height; ++y) {
    for (unsigned int x = 0; x < msg.info.width; ++x) {
      const std::size_t idx = static_cast<std::size_t>(y) * msg.info.width + x;
      if (idx >= msg.data.size()) {
        continue;
      }
      costmap_.setCost(x, y, occupancyToCost(msg.data[idx]));
    }
  }
}

void AssistedTeleop::setDefaultFootprint()
{
  const double half_x = footprint_length_ * 0.5;
  const double half_y = footprint_width_ * 0.5;

  footprint_.clear();
  footprint_.resize(4);

  footprint_[0].x = half_x;
  footprint_[0].y = half_y;

  footprint_[1].x = half_x;
  footprint_[1].y = -half_y;

  footprint_[2].x = -half_x;
  footprint_[2].y = -half_y;

  footprint_[3].x = -half_x;
  footprint_[3].y = half_y;

  has_footprint_ = true;
}

unsigned char AssistedTeleop::occupancyToCost(int8_t occupancy) const
{
  if (occupancy < 0) {
    return nav2_costmap_2d::NO_INFORMATION;
  }

  if (occupancy >= 99) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  const double normalized = std::clamp(static_cast<double>(occupancy), 0.0, 100.0) / 100.0;
  return static_cast<unsigned char>(std::round(normalized * 252.0));
}

geometry_msgs::msg::Twist AssistedTeleop::toTwist(const Eigen::Vector3f & vel) const
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = vel[0];
  msg.linear.y = vel[1];
  msg.angular.z = vel[2];
  return msg;
}

}  // namespace assisted_teleop

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<assisted_teleop::AssistedTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
