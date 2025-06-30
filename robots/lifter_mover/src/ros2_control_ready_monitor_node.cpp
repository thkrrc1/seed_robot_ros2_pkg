#include "rclcpp/rclcpp.hpp"
#include "pal_statistics_msgs/msg/statistics_names.hpp"
#include <cstdlib>
#include <string>
#include <sstream>

class Ros2ControlReadyMonitor : public rclcpp::Node {
public:
    Ros2ControlReadyMonitor()
    : Node("ros2_control_ready_monitor_node"), triggered_(false)
    {
        this->declare_parameter<std::string>("robot_pkg_path", "");

        robot_pkg_path_ = this->get_parameter("robot_pkg_path").as_string();

        auto qos = rclcpp::QoS(10).reliable().transient_local();
        sub_ = this->create_subscription<pal_statistics_msgs::msg::StatisticsNames>(
            "/controller_manager/introspection_data/names", qos,
            std::bind(&Ros2ControlReadyMonitor::callback, this, std::placeholders::_1)
        );
    }

private:
    void callback([[maybe_unused]] const pal_statistics_msgs::msg::StatisticsNames::SharedPtr msg) {
        if (!triggered_) {
            triggered_ = true;
            RCLCPP_INFO(this->get_logger(), "/controller_manager/introspection_data/names 受信");

            std::stringstream cmd;
            cmd << "ros2 launch " << robot_pkg_path_
                << "/launch/parts/bringup_controller_managers.launch.py "
                << "robot_pkg_path:=" << robot_pkg_path_ << " &";

            std::system(cmd.str().c_str());
        }
    }

    rclcpp::Subscription<pal_statistics_msgs::msg::StatisticsNames>::SharedPtr sub_;
    std::string robot_pkg_path_;
    bool triggered_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ros2ControlReadyMonitor>());
    rclcpp::shutdown();
    return 0;
}

