#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "std_msgs/msg/bool.hpp"


class LifecycleMonitor : public rclcpp::Node {
public:
  LifecycleMonitor(const std::vector<std::string> & target_nodes)
  : Node("lifecycle_monitor"), launched_(false)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    pub_ = this->create_publisher<std_msgs::msg::Bool>("whole_controllers_trigger", qos);
    for (const auto & name : target_nodes) {
      states_[name] = "unknown";
      auto sub = create_subscription<lifecycle_msgs::msg::TransitionEvent>(
        "/" + name + "/transition_event", 10,
        [this, name](lifecycle_msgs::msg::TransitionEvent::SharedPtr msg) {
          if (msg->goal_state.label == "active") {
            RCLCPP_INFO(get_logger(), "%s is now ACTIVE", name.c_str());
            states_[name] = "active";
            check_all_active();
          } else {
            RCLCPP_INFO(get_logger(), "%s transitioned to %s",
              name.c_str(), msg->goal_state.label.c_str());
          }
        });
      subs_.push_back(sub);
    }
  }

private:
  void check_all_active() {
    if (launched_) return;
    for (const auto & pair : states_) {
      if (pair.second != "active") return;
    }

    RCLCPP_INFO(get_logger(), "All nodes are ACTIVE. Publishing launch_trigger...");
    std_msgs::msg::Bool msg;
    msg.data = true;
    pub_->publish(msg);
    launched_ = true;
  }

  std::vector<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr> subs_;
  std::map<std::string, std::string> states_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  bool launched_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  std::vector<std::string> names = {"lifter_controller", "mechanum_controller", "aero_controller", "joint_state_broadcaster"};
  auto node = std::make_shared<LifecycleMonitor>(names);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

