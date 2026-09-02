#pragma once

#include <bitset>
#include <seed_ros2_controller/command_interface/other_command/other_command_controller_helper.hpp>

#include <aero_controller_msgs/msg/current.hpp>
#include <aero_controller_msgs/msg/current_parsed.hpp>
#include <aero_controller_msgs/msg/current_parsed_list.hpp>

#include "current.hpp"

#include <pluginlib/class_loader.hpp>
#include <seed_ros2_controller/stroke_converter/stroke_converter_base.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace aero_controller {

class CurrentController: public seed_ros2_controller::command_interface::OtherCommandControllerHelper {
public:
    CurrentController();

    ~CurrentController();

private:
    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    void execute(const std::string &protocol, const BuffRaw *buf_recv, BuffRaw *buf_send) override;

    void periodic_send();
    void stop_periodic_thread();

    std::vector<std::pair<uint8_t, uint8_t>> handleHeaders() override {
        return {{0xdf,0xfd}};
    }

    void parse_current(const aero_controller_msgs::msg::Current &current);

private:
    std::thread periodic_thread;
    rclcpp::Publisher<aero_controller_msgs::msg::Current>::SharedPtr  current_pub;
    rclcpp::Publisher<aero_controller_msgs::msg::CurrentParsedList>::SharedPtr  current_parsed_list_pub;
    aero_controller_msgs::msg::Current current_msg;
    aero_controller_msgs::msg::CurrentParsedList current_parsed_list_msg;
    std::atomic<bool> shutdown = false;
    std::atomic<bool> current_request_inflight = false;
    int ms = 1; // upper_msのみ
    BuffRaw current_buff;
    std::chrono::steady_clock::time_point last_send_time;

    int update_rate_default = 50;
    std::string convert_lib_name;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jstate_sub_;
    std::mutex jstate_mtx_;
    std::vector<std::string> jnames;
    sensor_msgs::msg::JointState jstate;
    std::vector<double> cur_angles;
    bool has_joint_state_ = false;
    pluginlib::ClassLoader<seed_converter::StrokeConverter> converter_loader_;
    std::shared_ptr<seed_converter::StrokeConverter> stroke_converter_ = nullptr;
};

}
