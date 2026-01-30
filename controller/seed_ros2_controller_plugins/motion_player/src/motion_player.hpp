#pragma once

#include "motion_player.hpp"
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>

#include <controller_interface/controller_interface.hpp>
#include <motion_player_msgs/action/play.hpp>

#include <seed_ros2_controller/command_interface/controller_base.hpp>
#include <seed_ros2_controller/command_interface/command_interface.hpp>
#include <seed_ros2_controller/command_interface/other_command/other_command_interface.hpp>


#include "motion_request_server.hpp"
#include "motion_base.hpp"
#include "rt_bridge.hpp"


namespace motion_player{
class MotionPlayer: public seed_ros2_controller::command_interface::ControllerBase<seed_ros2_controller::command_interface::OtherCommandInterface>
{
    using PlayAction = motion_player_msgs::action::Play;
public:

    MotionPlayer(){
    }

private:

    controller_interface::CallbackReturn on_init() override{
        using namespace std::chrono_literals;
        auto parameters_client = std::make_shared < rclcpp::SyncParametersClient > (get_node(), "/controller_manager");
        while (!parameters_client->wait_for_service(1s)) {
            RCLCPP_INFO(get_node()->get_logger(), "service not available, waiting again...");
            if(!rclcpp::ok()){
                return controller_interface::CallbackReturn::FAILURE;
            }
        }

        update_rate_default = parameters_client->get_parameter<int>("update_rate", 50);
        convert_lib_name = parameters_client->get_parameter<std::string>("converter_lib", "seed_converter/NoidLifterMover");
        
        return controller_interface::CallbackReturn::SUCCESS;
    };

    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override{
        req_server.init(get_node(), &swp_buff_req, &swp_buff_resp, &stop_request, convert_lib_name);
        return controller_interface::CallbackReturn::SUCCESS;
    };

    void activate_hardware(seed_ros2_controller::command_interface::OtherCommandInterface& hw) override{
        for (auto &msid_str : hw.get_names()) {
            handles.push_back(hw.get_handle(msid_str));
            ms_ids.push_back(std::stoi(msid_str));
        }

        auto update_rate = get_update_rate();
        if (update_rate <= 0) {
            update_rate = update_rate_default;
        }
        req_server.update_hardware(hw, 1.0 / update_rate);
    }

    void release_hardware() override {
        handles.clear();
        ms_ids.clear();
    }


    controller_interface::return_type  update(const rclcpp::Time &time, const rclcpp::Duration &period) override{
        if(!curdata){
            curdata = swp_buff_req.readFromB();
            if(curdata){
                step = curdata->step;
            }
        }else{
            ++step;
        }


        bool running = false;
        if(curdata){
            running = true;
        }

        if(stop_request.load()){
            curdata = nullptr;
            stop_request.store(false);

            //TODO 停止軌道
        }

        while (curdata) {
            if (step == curdata->step) {
                int msid = curdata->msid;
                for(size_t idx = 0;idx<ms_ids.size();++idx){
                    if(msid == ms_ids[idx]){
                        handles[idx].send_cmd(&curdata->rawdata);
                    }
                }
                curdata = swp_buff_req.readFromB(true);
            }else{
                break;
            }
        }

        if(running){
            Response resp = {step,!curdata};
            swp_buff_resp.writeFromA(resp);
        }

        return controller_interface::return_type::OK;
    }

private:
    std::vector<int> ms_ids;
    std::vector<seed_ros2_controller::command_interface::OtherCommandHandle> handles;
    rclcpp_action::Server<PlayAction>::SharedPtr action_server_;

    MotionRequestServer req_server;

    reqBuffType swp_buff_req;
    respBuffType swp_buff_resp;
    Request* curdata = nullptr;

    std::atomic<bool> stop_request = false;
    int step = 0;

    int update_rate_default = 50;
    std::string convert_lib_name; 
};

}
