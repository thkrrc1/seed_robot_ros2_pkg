#include <iostream>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <seed_ros2_controller/command_interface/status/robot_status.hpp>
#include <seed_ros2_controller/command_interface/other_command/other_command_interface.hpp>
#include <seed_ros2_controller/command_interface/status/status_interface.hpp>

#include "rt_logger/logger.hpp"
#include "rt_logger/spdlog_logger/spdlog_logger.hpp"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"

#include "robot_driver.hpp"



namespace seed_ros2_controller {
class RobotHardware: public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS (RobotHardware);

    ~RobotHardware();

private:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    std::vector<double> pos_send_;
    std::vector<double> vel_send_;

    std::vector<double> pos_recv_;
    std::vector<double> vel_recv_;

    Status status_recv_;

    std::vector<BuffList> other_cmds_recv_;
    std::vector<BuffList> other_cmds_send_;

    RobotDriver* driver_ = nullptr;

    seed_ros2_controller::command_interface::OtherCommandInterface other_interf;
    seed_ros2_controller::command_interface::StatusInterface status_interf;

    LOGGER_INIT
};

}
