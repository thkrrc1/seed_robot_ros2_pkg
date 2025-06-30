#pragma once

#include <condition_variable>
#include <mutex>

#include <config_controller_msgs/srv/ms_config_read.hpp>
#include <config_controller_msgs/srv/ms_config_write.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <std_srvs/srv/empty.hpp>

#include <seed_ros2_controller/command_interface/other_command/other_command_controller_helper.hpp>



namespace config_controller {

class ConfigController: public seed_ros2_controller::command_interface::OtherCommandControllerHelper {
public:
    ConfigController();

private:
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    bool configWrite(std::shared_ptr<config_controller_msgs::srv::MsConfigWrite::Request> req, std::shared_ptr<config_controller_msgs::srv::MsConfigWrite::Response> res);
    bool configRead(std::shared_ptr<config_controller_msgs::srv::MsConfigRead::Request> req, std::shared_ptr<config_controller_msgs::srv::MsConfigRead::Response> res);

    std::vector<std::pair<uint8_t, uint8_t>> handleHeaders() override {
        return {{0xcf,0xfc},{0xfc,0xcf}};
    }

    bool configBegin(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    bool configEnd(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);

    void execute(const std::string& protocol, const BuffRaw *buf_recv, BuffRaw *buf_send) override;

private:
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr config_begin_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr config_end_srv_;
    rclcpp::Service<config_controller_msgs::srv::MsConfigWrite>::SharedPtr config_write_srv_;
    rclcpp::Service<config_controller_msgs::srv::MsConfigRead>::SharedPtr config_read_srv_;

//    rclcpp::Client<config_controller_msgs::srv::MsConfigRead>::SharedPtr othercmd_only_client_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_client_;
    std::vector<std::string> halted_controllers_;
    rclcpp::CallbackGroup::SharedPtr grp_;

    BuffRaw recv_buff_raw;

    std::mutex mtx;
    std::condition_variable cond;

    std::vector<int> ms_need_reset;
};

}
