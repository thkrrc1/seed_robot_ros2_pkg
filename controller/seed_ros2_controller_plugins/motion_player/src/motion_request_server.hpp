#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <queue>

#include <pluginlib/class_loader.hpp>
#include <seed_ros2_controller/stroke_converter/stroke_converter_base.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <motion_player_msgs/srv/load.hpp>
#include <motion_player_msgs/action/play.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>

#include "motion_base.hpp"
#include "media_player.hpp"
#include "rt_bridge.hpp"
#include "arm_info.hpp"
#include "urdf_helper.hpp"

#include <seed_ros2_controller/command_interface/other_command/other_command_interface.hpp>

namespace motion_player {

struct JointTarget{
    std::vector<double> tgtpos;
};

class MotionRequestServer
{
    using PlayAction = motion_player_msgs::action::Play;
    typedef rclcpp_action::Server<PlayAction> PlayActionServer;
    typedef rclcpp_action::Server<PlayAction>::SharedPtr PlayActionServerPtr;

public:
    MotionRequestServer() : converter_loader_("seed_ros2_controller", "seed_converter::StrokeConverter"){
    }

    ~MotionRequestServer();

    void init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, reqBuffType *req_buff, respBuffType *resp_buff, std::atomic<bool> *stop_request, const std::string &converter_lib);

    void update_hardware(const seed_ros2_controller::command_interface::OtherCommandInterface &hw, double control_period_sec);

private:
    rclcpp_action::GoalResponse goal_received_callback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const PlayAction::Goal> goal);

    rclcpp_action::CancelResponse goal_cancelled_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PlayAction>> goal_handle);

    void goal_accepted_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<PlayAction>> goal_handle);

    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<PlayAction>> goal_handle);

    bool load(const std::shared_ptr<motion_player_msgs::srv::Load::Request> request, std::shared_ptr<motion_player_msgs::srv::Load::Response> response);

    void sendThread();

    sensor_msgs::msg::JointState getCurrentJointState();

    double getIntroductionTrajMinSec(int start_step, const sensor_msgs::msg::JointState &jstate);

    motion_editor2::ArmTarget2 calcIntroductionTraj(int step, int step_st,int step_end, const sensor_msgs::msg::JointState &jstate,const std::vector<motion_editor2::ArmTarget2> &arm_tgts);

private:
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_client_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
    rclcpp::Service<motion_player_msgs::srv::Load>::SharedPtr load_srv;
    PlayActionServerPtr play_as = nullptr;
//
    reqBuffType *req_buff = nullptr;
    respBuffType *resp_buff = nullptr;
//
    std::vector<std::string> joint_names;
    std::vector<std::string> deactivated_controllers;
    double control_period_sec = 0.02;
    ArmInfoList arm_info;

    pluginlib::ClassLoader<seed_converter::StrokeConverter> converter_loader_;
    std::shared_ptr<seed_converter::StrokeConverter> stroke_converter_ = nullptr;

    std::atomic<bool> shutdown = false;
    std::atomic<bool> cancel = false;
    std::thread send_thread;
    std::mutex mtx;
    std::condition_variable cond;
//
    motion_editor2::MotionBase2* cur_motion = nullptr;
    motion_editor2::MotionIterator2::Ptr cur_miter = nullptr;
    JointTarget jtarget;

    double dt_sec = 0.02;
    int start_step = 0;

    std::atomic<bool>* stop_request = nullptr;
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;

    std::map<std::string,JointLimitType> joint_limits;

    motion_editor2::MediaPlayer media_player;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr desc_sub_;
};
}
