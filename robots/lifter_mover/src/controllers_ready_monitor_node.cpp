#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cstdlib>
#include <string>
#include <sstream>

class ControllersReadyMonitor : public rclcpp::Node {
public:
    ControllersReadyMonitor()
    : Node("gazebo_controllers_ready_monitor"), triggered_(false)
    {
        this->declare_parameter<std::string>("robot_pkg_path", "");
        this->declare_parameter<bool>("simulation", true);
        this->declare_parameter<bool>("slam", false);
        this->declare_parameter<bool>("use_localization", true);
        this->declare_parameter<std::string>("map", "");

        robot_pkg_path_ = this->get_parameter("robot_pkg_path").as_string();
        simulation_ = this->get_parameter("simulation").as_bool();
        slam_ = this->get_parameter("slam").as_bool();
        use_localization_ = this->get_parameter("use_localization").as_bool();
        map_ = this->get_parameter("map").as_string();

        auto qos = rclcpp::QoS(10).reliable().transient_local();
        sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/whole_controllers_trigger", qos,
            std::bind(&ControllersReadyMonitor::callback, this, std::placeholders::_1)
        );
    }

private:
    void callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!triggered_ && msg->data) {
            triggered_ = true;
            RCLCPP_INFO(this->get_logger(), "/controllers_trigger received, launching navigation");
  
            std::stringstream nav_cmd;
            if (simulation_) {
                nav_cmd << "ros2 launch " << robot_pkg_path_
                        << "/launch/parts/bringup_gz_navigation.launch.py "
                        << "robot_pkg_path:=" << robot_pkg_path_ << " "
                        << "slam:=" << (slam_ ? "True" : "False") << " "
                        << "use_sim_time:=True "
                        << "use_localization:=" << (use_localization_ ? "True" : "False") << " "
                        << "map:=" << map_ << " &";
            } else {
                nav_cmd << "ros2 launch " << robot_pkg_path_
                        << "/launch/parts/bringup_navigation.launch.py "
                        << "robot_pkg_path:=" << robot_pkg_path_ << " "
                        << "slam:=" << (slam_ ? "True" : "False") << " "
                        << "use_sim_time:=False "
                        << "use_localization:=" << (use_localization_ ? "True" : "False") << " "
                        << "map:=" << map_ << " &";
            }
            std::system(nav_cmd.str().c_str());
            
            std::string tf_cmd =
                "ros2 run tf2_ros static_transform_publisher "
                "--x 0 --y 0 --z 0 "
                "--qx 0 --qy 0 --qz 0 --qw 1 "
                "--frame-id map --child-frame-id odom &";
            std::system(tf_cmd.c_str());

            
            if (slam_ && !simulation_) {
                std::stringstream lidar_cmd;
                lidar_cmd << "ros2 launch " << robot_pkg_path_
                          << "/launch/parts/bringup_lidar.launch.py "
                          << "robot_pkg_path:=" << robot_pkg_path_ << " &";
                std::system(lidar_cmd.str().c_str());
                
                std::stringstream teleop_cmd;
                teleop_cmd << "ros2 launch " << robot_pkg_path_
                           << "/launch/parts/bringup_teleop.launch.py "
                           << "robot_pkg_path:=" << robot_pkg_path_ << " &";
                std::system(teleop_cmd.str().c_str());
            }
            
            if (slam_ && simulation_) {              
                std::stringstream teleop_cmd;
                teleop_cmd << "ros2 launch " << robot_pkg_path_
                           << "/launch/parts/bringup_gz_teleop.launch.py "
                           << "robot_pkg_path:=" << robot_pkg_path_ << " &";
                std::system(teleop_cmd.str().c_str());
            }
            
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
    std::string robot_pkg_path_;
    bool simulation_;
    bool slam_;
    bool use_localization_;
    std::string map_;
    bool triggered_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllersReadyMonitor>());
    rclcpp::shutdown();
    return 0;
}
