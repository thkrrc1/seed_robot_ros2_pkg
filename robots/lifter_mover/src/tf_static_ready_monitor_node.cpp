#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include <unistd.h> 
#include <sys/types.h>
#include <cstdlib>
#include <vector>
#include <string>

class TFStaticReadyMonitor : public rclcpp::Node
{
public:
  TFStaticReadyMonitor()
  : Node("tf_static_ready_monitor"), triggered_(false)
  {
    this->declare_parameter<std::string>("param_path", "");
    param_path_ = this->get_parameter("param_path").as_string();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();

    subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "/tf_static_ready", qos,
      std::bind(&TFStaticReadyMonitor::callback, this, std::placeholders::_1));
  }

private:
  void callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && !triggered_) {
      triggered_ = true;
      RCLCPP_INFO(this->get_logger(), "/tf_static_ready received – launching controller_manager");

      std::vector<std::string> args_str = {
        "ros2", "run",
        "controller_manager", "ros2_control_node",
        "--ros-args",
        "--params-file", param_path_,
        "--remap", "~/robot_description:=/robot_description",
        "--remap", "/mechanum_controller/cmd_vel_nav:=/cmd_vel_nav"
      };

      std::vector<char*> args;
      for (auto& s : args_str) {
        args.push_back(const_cast<char*>(s.c_str()));
      }
      args.push_back(nullptr);

      pid_t pid = fork();
      if (pid == 0) {
        execvp("ros2", args.data());
        std::perror("execvp failed");
        std::exit(1);
      } else if (pid > 0) {
        RCLCPP_INFO(this->get_logger(), "controller_manager launched with PID %d", pid);
      } else {
        RCLCPP_ERROR(this->get_logger(), "fork failed");
      }
    }
  }

  
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
  std::string param_path_;
  bool triggered_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFStaticReadyMonitor>());
  rclcpp::shutdown();
  return 0;
}
