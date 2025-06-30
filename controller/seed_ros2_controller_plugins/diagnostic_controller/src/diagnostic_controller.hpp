#pragma once

#include <controller_interface/controller_interface.hpp>

#include <seed_ros2_controller/command_interface/controller_base.hpp>
#include <seed_ros2_controller/command_interface/command_interface.hpp>
#include <seed_ros2_controller/command_interface/status/status_interface.hpp>
#include <aero_controller_msgs/srv/servo.hpp>


#include <diagnostic_updater/diagnostic_updater.hpp>
#include "libs/swap_buffer.hpp"

namespace diagnostic_controller {

class DiagnosticController: public seed_ros2_controller::command_interface::ControllerBase<seed_ros2_controller::command_interface::StatusInterface>{
public:
    DiagnosticController() = default;
    ~DiagnosticController() = default;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    void activate_hardware(seed_ros2_controller::command_interface::StatusInterface& hw) override;
    void release_hardware() override;
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    void setDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
private:
    seed_ros2_controller::command_interface::StatusHandle handle;
    std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_ = nullptr;

    SwapBuffer<Status, 1> swp_buff_recv;
    rclcpp::Client<aero_controller_msgs::srv::Servo>::SharedPtr servo_client_;
};
}
