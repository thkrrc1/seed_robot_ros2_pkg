#pragma once

#include <bitset>
#include <seed_ros2_controller/command_interface/other_command/other_command_controller_helper.hpp>
#include <aero_controller_msgs/msg/status.hpp>
#include <aero_controller_msgs/msg/status_parsed.hpp>


namespace aero_controller {

class StatusController: public seed_ros2_controller::command_interface::OtherCommandControllerHelper {
public:
    StatusController();

    ~StatusController();

private:
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    void execute(const std::string &protocol, const BuffRaw *buf_recv, BuffRaw *buf_send) override;

    void periodic_send();

    std::vector<std::pair<uint8_t, uint8_t>> handleHeaders() override {
        return {{0xdf,0xfd}};
    }

    void parse_status(const aero_controller_msgs::msg::Status &status);

private:
    std::thread periodic_thread;
    rclcpp::Publisher<aero_controller_msgs::msg::Status>::SharedPtr status_pub;
    rclcpp::Publisher<aero_controller_msgs::msg::StatusParsed>::SharedPtr status_parsed_pub;
    aero_controller_msgs::msg::Status status_msg;
    aero_controller_msgs::msg::StatusParsed status_parsed_msg;
    bool status_parse = true;
    std::atomic<bool> shutdown = false;
};

}
