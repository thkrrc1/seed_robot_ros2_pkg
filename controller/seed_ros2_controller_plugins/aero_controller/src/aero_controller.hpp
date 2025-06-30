#pragma once

#include <seed_ros2_controller/command_interface/other_command/other_command_controller_helper.hpp>
#include <aero_controller_msgs/srv/servo.hpp>
#include <aero_controller_msgs/srv/set_current.hpp>
#include <aero_controller_msgs/srv/run_script.hpp>
#include <aero_controller_msgs/srv/get_version.hpp>
#include <aero_controller_msgs/srv/get_ad.hpp>
#include <aero_controller_msgs/srv/can_through.hpp>


#include "script_controller.hpp"
#include "verget_controller.hpp"
#include "adget_controller.hpp"

namespace aero_controller {

class AeroController: public seed_ros2_controller::command_interface::OtherCommandControllerHelper {
public:
    AeroController();
    ~AeroController();

    void sendScriptCommand(const ScriptContext& context);
    void sendDioCommand();
    void sendVerGetCommand(uint8_t msid, uint8_t mcid);
    void sendAdGetCommand(int ad_no);
private:
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    std::vector<std::pair<uint8_t, uint8_t>> handleHeaders() override {
        return {{0xdf,0xfd}};
    }
    void execute(int msid, const std::string &protocol, const BuffRaw *buf_recv, BuffRaw *buf_send) override;

    bool servo(std::shared_ptr<aero_controller_msgs::srv::Servo::Request> req, std::shared_ptr<aero_controller_msgs::srv::Servo::Response> res);
    bool setCurrent(std::shared_ptr<aero_controller_msgs::srv::SetCurrent::Request> req, std::shared_ptr<aero_controller_msgs::srv::SetCurrent::Response> res);
    bool runScript(std::shared_ptr<aero_controller_msgs::srv::RunScript::Request> req, std::shared_ptr<aero_controller_msgs::srv::RunScript::Response> res);
    bool getVersion(std::shared_ptr<aero_controller_msgs::srv::GetVersion::Request> req, std::shared_ptr<aero_controller_msgs::srv::GetVersion::Response> res);
    bool getAd(std::shared_ptr<aero_controller_msgs::srv::GetAd::Request> req, std::shared_ptr<aero_controller_msgs::srv::GetAd::Response> res);
    bool canThrough(std::shared_ptr<aero_controller_msgs::srv::CanThrough::Request> req, std::shared_ptr<aero_controller_msgs::srv::CanThrough::Response> res);

    ScriptContext createScriptContext(const aero_controller_msgs::srv::RunScript::Request &req) const;
    aero_controller_msgs::srv::RunScript::Request createScriptRequest(const ScriptContext &context) const;
    aero_controller_msgs::srv::RunScript::Response createScriptResponse(const ScriptContext &context) const;

    void cmdSendThread();

private:
    rclcpp::Service<aero_controller_msgs::srv::Servo>::SharedPtr servo_srv;
    rclcpp::Service<aero_controller_msgs::srv::SetCurrent>::SharedPtr scur_srv;
    rclcpp::Service<aero_controller_msgs::srv::RunScript>::SharedPtr rscr_srv;
    rclcpp::Service<aero_controller_msgs::srv::GetVersion>::SharedPtr verget_srv;
    rclcpp::Service<aero_controller_msgs::srv::GetAd>::SharedPtr adget_srv;
    rclcpp::Service<aero_controller_msgs::srv::CanThrough>::SharedPtr canthrough_srv;

    ScriptController scr_ctrl;
    VerGetController verget_ctrl;
    AdGetController adget_ctrl;

    std::atomic<bool> running = true;
    std::thread cmd_thread;
    std::mutex cmd_mtx;
    bool cmd_req = false;
};

}
