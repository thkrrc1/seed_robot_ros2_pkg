#pragma once

#include <bitset>
#include <seed_ros2_controller/command_interface/other_command/other_command_controller_helper.hpp>

#include <aero_controller_msgs/msg/robot_status.hpp>

#include <aero_controller_msgs/msg/robot_status_parsed.hpp>
#include <aero_controller_msgs/msg/robot_status_parsed_list.hpp>

namespace aero_controller {

class RobotStatusController: public seed_ros2_controller::command_interface::OtherCommandControllerHelper {
public:
    RobotStatusController();

    ~RobotStatusController();

private:
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    void execute(const std::string &protocol, const BuffRaw *buf_recv, BuffRaw *buf_send) override;

    void periodic_send();

    std::vector<std::pair<uint8_t, uint8_t>> handleHeaders() override {
        return {{0xdf,0xfd}};
    }
    
    bool activate() override final{
        // auto mslist = getMsList();
        int ms = 1;   //ros1移植今後修正予定
        auto jointsname = getJointNames(ms);
        for(size_t jidx = 0; jidx < jointsname.size(); ++jidx)
        {
            if(jointsname[jidx].empty()){
                continue;
            }
            active_aero_idx_list.push_back(jidx);
        } 
        return true ;       
    };
    void parse_status(const aero_controller_msgs::msg::RobotStatus &status);


private:
    std::thread periodic_thread;
    rclcpp::Publisher<aero_controller_msgs::msg::RobotStatus>::SharedPtr  status_pub;
    rclcpp::Publisher<aero_controller_msgs::msg::RobotStatusParsedList>::SharedPtr  status_parsed_pub;
    std::vector<int> active_aero_idx_list;
    std::vector<rclcpp::Parameter> joints_param;
    aero_controller_msgs::msg::RobotStatus status_msg;
    aero_controller_msgs::msg::RobotStatusParsedList status_parsed_msg;
    std::atomic<bool> shutdown = false;
};

}
