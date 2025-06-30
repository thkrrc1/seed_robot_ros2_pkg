#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "realtime_tools/realtime_box.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "controller_interface/controller_interface.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <mechanum_controller/mechanum_controller_params.hpp>
#include "kinematics.hpp"
#include "odometry.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "speed_limiter.hpp"
#include "rt_logger/logger.hpp"

namespace mechanum_controller {

class MechanumController: public controller_interface::ControllerInterface {
public:

    //一番最初に呼ばれる
    //これが呼ばれたときには、まだcontroller_settingsは反映されていない。
    controller_interface::CallbackReturn on_init() override {
        param_listener = std::make_shared < mechanum_controller::ParamListener > (get_node());
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override {
        params = param_listener->get_params();
        odom_pub = get_node()->create_publisher < nav_msgs::msg::Odometry > ("~/odometry", rclcpp::SystemDefaultsQoS());
        odom_pub_rt = std::make_unique < realtime_tools::RealtimePublisher < nav_msgs::msg::Odometry >> (odom_pub);

        odom_pub_rt->lock();
        odom_pub_rt->msg_.header.stamp = get_node()->now();
        odom_pub_rt->msg_.header.frame_id = params.frame.odom_frame_id;
        odom_pub_rt->msg_.child_frame_id = params.frame.base_frame_id;
        odom_pub_rt->msg_.pose.pose.position.z = 0;
        odom_pub_rt->msg_.twist = geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);
        odom_pub_rt->unlock();

        tf_odom_pub = get_node()->create_publisher < tf2_msgs::msg::TFMessage > ("/tf", rclcpp::SystemDefaultsQoS());
        tf_odom_pub_rt = std::make_unique < realtime_tools::RealtimePublisher < tf2_msgs::msg::TFMessage >> (tf_odom_pub);

        tf_odom_pub_rt->lock();
        tf_odom_pub_rt->msg_.transforms.resize(1);
        tf_odom_pub_rt->msg_.transforms[0].header.stamp = get_node()->now();
        tf_odom_pub_rt->msg_.transforms[0].header.frame_id = params.frame.odom_frame_id;
        tf_odom_pub_rt->msg_.transforms[0].child_frame_id = params.frame.base_frame_id;
        tf_odom_pub_rt->msg_.transforms[0].transform.translation.z = 0.0;
        tf_odom_pub_rt->unlock();

        cmd_vel_timeout = rclcpp::Duration::from_seconds(params.time.cmd_vel_timeout);
        teleop_sub_timeout = rclcpp::Duration::from_seconds(params.time.teleop_sub_timeout);

        vel_sub_nav = get_node()->create_subscription < geometry_msgs::msg::Twist > ("~/cmd_vel_nav", rclcpp::SystemDefaultsQoS(), std::bind(&MechanumController::cmdVelNavCallback, this, std::placeholders::_1));
        vel_sub_teleop = get_node()->create_subscription < geometry_msgs::msg::Twist > ("~/cmd_vel_teleop", rclcpp::SystemDefaultsQoS(), std::bind(&MechanumController::cmdVelTeleopCallback, this, std::placeholders::_1));
        last_teleop_sub_time = get_node()->get_clock()->now();

        kinema.init(params.size.wheel_radius, params.size.tread, params.size.wheel_base);

        bool enable_wheel_lim = params.speed.wheel.enable;
        limiter_front_left = SpeedLimiter(enable_wheel_lim,enable_wheel_lim,enable_wheel_lim,
                -params.speed.wheel.max_velocity,
                 params.speed.wheel.max_velocity,
                -params.speed.wheel.max_acceleration,
                 params.speed.wheel.max_acceleration,
                -params.speed.wheel.max_jerk,
                 params.speed.wheel.max_jerk);

        limiter_front_right = SpeedLimiter(enable_wheel_lim,enable_wheel_lim,enable_wheel_lim,
                -params.speed.wheel.max_velocity,
                 params.speed.wheel.max_velocity,
                -params.speed.wheel.max_acceleration,
                 params.speed.wheel.max_acceleration,
                -params.speed.wheel.max_jerk,
                 params.speed.wheel.max_jerk);

        limiter_rear_left = SpeedLimiter(enable_wheel_lim,enable_wheel_lim,enable_wheel_lim,
                -params.speed.wheel.max_velocity,
                 params.speed.wheel.max_velocity,
                -params.speed.wheel.max_acceleration,
                 params.speed.wheel.max_acceleration,
                -params.speed.wheel.max_jerk,
                 params.speed.wheel.max_jerk);

        limiter_rear_right = SpeedLimiter(enable_wheel_lim,enable_wheel_lim,enable_wheel_lim,
                -params.speed.wheel.max_velocity,
                 params.speed.wheel.max_velocity,
                -params.speed.wheel.max_acceleration,
                 params.speed.wheel.max_acceleration,
                -params.speed.wheel.max_jerk,
                 params.speed.wheel.max_jerk);

        bool enable_planar_lim = params.speed.planar.enable;
        limiter_x = SpeedLimiter(enable_planar_lim,enable_planar_lim,enable_planar_lim,
                -params.speed.planar.x.max_velocity,
                 params.speed.planar.x.max_velocity,
                -params.speed.planar.x.max_acceleration,
                 params.speed.planar.x.max_acceleration,
                -params.speed.planar.x.max_jerk,
                 params.speed.planar.x.max_jerk);
        limiter_y = SpeedLimiter(enable_planar_lim,enable_planar_lim,enable_planar_lim,
                -params.speed.planar.y.max_velocity,
                 params.speed.planar.y.max_velocity,
                -params.speed.planar.y.max_acceleration,
                 params.speed.planar.y.max_acceleration,
                -params.speed.planar.y.max_jerk,
                 params.speed.planar.y.max_jerk);
        limiter_th = SpeedLimiter(enable_planar_lim,enable_planar_lim,enable_planar_lim,
                -params.speed.planar.angular.max_velocity,
                 params.speed.planar.angular.max_velocity,
                -params.speed.planar.angular.max_acceleration,
                 params.speed.planar.angular.max_acceleration,
                -params.speed.planar.angular.max_jerk,
                 params.speed.planar.angular.max_jerk);

        return CallbackReturn::SUCCESS;
    }

    //コマンドの送信先を設定する
    controller_interface::InterfaceConfiguration command_interface_configuration() const override {
        controller_interface::InterfaceConfiguration config;

        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.push_back(params.joints.front_left_wheel + "/" + hardware_interface::HW_IF_VELOCITY);
        config.names.push_back(params.joints.front_right_wheel + "/" + hardware_interface::HW_IF_VELOCITY);
        config.names.push_back(params.joints.rear_left_wheel + "/" + hardware_interface::HW_IF_VELOCITY);
        config.names.push_back(params.joints.rear_right_wheel + "/" + hardware_interface::HW_IF_VELOCITY);

        return config;
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.push_back(params.joints.front_left_wheel + "/" + hardware_interface::HW_IF_VELOCITY);
        config.names.push_back(params.joints.front_right_wheel + "/" + hardware_interface::HW_IF_VELOCITY);
        config.names.push_back(params.joints.rear_left_wheel + "/" + hardware_interface::HW_IF_VELOCITY);
        config.names.push_back(params.joints.rear_right_wheel + "/" + hardware_interface::HW_IF_VELOCITY);
        return config;
    }

    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override {
        return CallbackReturn::SUCCESS;
    }

    void cmdVelRequest(const rclcpp::Time & now, const geometry_msgs::msg::Twist &command) {
        if (std::isnan(command.angular.z) || std::isnan(command.linear.x) || std::isnan(command.linear.y)) {
            LOG_WARN_STREAM() << "Received NaN in geometry_msgs::Twist. Ignoring command." << LOG_END;
            return;
        }

        //コマンドを、リアルタイムとの連携用バッファにつめ直す
        geometry_msgs::msg::TwistStamped twist_stamped;
        twist_stamped.twist = command;
        twist_stamped.header.stamp = now;
        vel_msg.writeFromNonRT(twist_stamped);
    }

    void cmdVelNavCallback(const geometry_msgs::msg::Twist::SharedPtr command) {
        std::lock_guard lock(cmdvel_mtx);
        auto now = get_node()->get_clock()->now();
        if (now - last_teleop_sub_time > teleop_sub_timeout) {
            cmdVelRequest(now, *command);
        }
    }

    void cmdVelTeleopCallback(const geometry_msgs::msg::Twist::SharedPtr command) {
        std::lock_guard lock(cmdvel_mtx);
        auto now = get_node()->get_clock()->now();
        last_teleop_sub_time = now;
        cmdVelRequest(now, *command);
    }


    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override {
        double period_sec = period.seconds();

        WheelVelocity command;
        auto current_msg = vel_msg.readFromRT();
        const auto age_of_last_command = time - current_msg->header.stamp;
        mechanum_controller::Velocity tgt_vel;
        if ((age_of_last_command < cmd_vel_timeout || cmd_vel_timeout == rclcpp::Duration::from_seconds(0))
            && (!std::isnan(current_msg->twist.linear.x) && !std::isnan(current_msg->twist.linear.y) && !std::isnan(current_msg->twist.angular.z))) {
                tgt_vel.vx = current_msg->twist.linear.x;
                tgt_vel.vy = current_msg->twist.linear.y;
                tgt_vel.vth = current_msg->twist.angular.z;
        }

        //平面でのリミット
        limiter_x.limit(tgt_vel.vx,tgt_vel_1stlast.vx,tgt_vel_2ndlast.vx,period_sec);
        limiter_y.limit(tgt_vel.vy,tgt_vel_1stlast.vy,tgt_vel_2ndlast.vy,period_sec);
        limiter_th.limit(tgt_vel.vth,tgt_vel_1stlast.vth,tgt_vel_2ndlast.vth,period_sec);
        tgt_vel_2ndlast = tgt_vel_1stlast;
        tgt_vel_1stlast = tgt_vel;

        kinema.velocityToWheel(tgt_vel.vx, tgt_vel.vy, tgt_vel.vth, command.front_left, command.front_right, command.rear_left, command.rear_right);

#if 1 //各軸の同期を取りつつ、速度制限
        double check1 = limiter_front_left.check(command.front_left, cmd_1stlast.front_left, cmd_2ndlast.front_left, period_sec);
        double check2 = limiter_front_right.check(command.front_right, cmd_1stlast.front_right, cmd_2ndlast.front_right, period_sec);
        double check3 = limiter_rear_left.check(command.rear_left, cmd_1stlast.rear_left, cmd_2ndlast.rear_left, period_sec);
        double check4 = limiter_rear_right.check(command.rear_right, cmd_1stlast.rear_right, cmd_2ndlast.rear_right, period_sec);

        double min = std::min( { check1, check2, check3, check4 });
        double max = std::max( { check1, check2, check3, check4 });
        if (min > 0) { //目標と同じ向きの場合は、速度を小さくする方を選ぶ
            command.front_left *= min;
            command.front_right *= min;
            command.rear_left *= min;
            command.rear_right *= min;
        } else { //目標と逆向きの場合
            command.front_left *= max;
            command.front_right *= max;
            command.rear_left *= max;
            command.rear_right *= max;
        }
#endif
        limiter_front_left.limit(command.front_left, cmd_1stlast.front_left, cmd_2ndlast.front_left, period_sec);
        limiter_front_right.limit(command.front_right, cmd_1stlast.front_right, cmd_2ndlast.front_right, period_sec);
        limiter_rear_left.limit(command.rear_left, cmd_1stlast.rear_left, cmd_2ndlast.rear_left, period_sec);
        limiter_rear_right.limit(command.rear_right, cmd_1stlast.rear_right, cmd_2ndlast.rear_right, period_sec);


        cmd_2ndlast = cmd_1stlast;
        cmd_1stlast = command;

        bool success_fl = command_interfaces_[FRONT_LEFT].set_value(command.front_left);
        if (!success_fl) {
            RCLCPP_WARN(get_node()->get_logger(), "Failed to set value for FRONT_LEFT");
        }
        bool success_fr = command_interfaces_[FRONT_RIGHT].set_value(command.front_right);
        if (!success_fr) {
            RCLCPP_WARN(get_node()->get_logger(), "Failed to set value for FRONT_RIGHT");
        }
        bool success_rl = command_interfaces_[REAR_LEFT].set_value(command.rear_left);
        if (!success_rl) {
            RCLCPP_WARN(get_node()->get_logger(), "Failed to set value for REAR_LEFT");
        }
        bool success_rr = command_interfaces_[REAR_RIGHT].set_value(command.rear_right);
        if (!success_rr) {
            RCLCPP_WARN(get_node()->get_logger(), "Failed to set value for REAR_RIGHT");
        }

        WheelVelocity feedback;
        mechanum_controller::Velocity cur_vel;
        //※分解能などの問題から、応答は速度にはノイズが乗る点に注意。
        auto value_opt_fl = state_interfaces_[FRONT_LEFT].get_optional();
        if (value_opt_fl.has_value()) {
            feedback.front_left = value_opt_fl.value();
        } else {
            RCLCPP_WARN(get_node()->get_logger(), "No value available from FRONT_LEFT state interface");
        }
        auto value_opt_fr = state_interfaces_[FRONT_RIGHT].get_optional();
        if (value_opt_fr.has_value()) {
            feedback.front_right = value_opt_fr.value();
        } else {
            RCLCPP_WARN(get_node()->get_logger(), "No value available from FRONT_RIGHT state interface");
        }
        auto value_opt_rl = state_interfaces_[REAR_LEFT].get_optional();
        if (value_opt_rl.has_value()) {
            feedback.rear_left = value_opt_rl.value();
        } else {
            RCLCPP_WARN(get_node()->get_logger(), "No value available from REAR_LEFT state interface");
        }
        auto value_opt_rr = state_interfaces_[REAR_RIGHT].get_optional();
        if (value_opt_rr.has_value()) {
            feedback.rear_right = value_opt_rr.value();
        } else {
            RCLCPP_WARN(get_node()->get_logger(), "No value available from REAR_RIGHT state interface");
        }

        if (!std::isnan(feedback.front_left) && !std::isnan(feedback.front_right) && !std::isnan(feedback.rear_left) && !std::isnan(feedback.rear_right)) {
            kinema.wheelToVelocity(feedback.front_left, feedback.front_right, feedback.rear_left, feedback.rear_right, cur_vel.vx, cur_vel.vy, cur_vel.vth);
            odom.update(cur_vel, period_sec);
        }
        auto global_pose = odom.get_pose();

        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, global_pose.th);
        if (odom_pub_rt->trylock()) {
            odom_pub_rt->msg_.header.stamp = time;
            odom_pub_rt->msg_.pose.pose.position.x = global_pose.x;
            odom_pub_rt->msg_.pose.pose.position.y = global_pose.y;
            odom_pub_rt->msg_.pose.pose.position.z = 0;
            odom_pub_rt->msg_.pose.pose.orientation = tf2::toMsg(orientation);
            odom_pub_rt->unlockAndPublish();
        }

        if (tf_odom_pub_rt->trylock()) {
            tf_odom_pub_rt->msg_.transforms.front().header.stamp = time;
            tf_odom_pub_rt->msg_.transforms.front().transform.translation.x = global_pose.x;
            tf_odom_pub_rt->msg_.transforms.front().transform.translation.y = global_pose.y;
            tf_odom_pub_rt->msg_.transforms.front().transform.translation.z = 0;
            tf_odom_pub_rt->msg_.transforms.front().transform.rotation = tf2::toMsg(orientation);
            tf_odom_pub_rt->unlockAndPublish();
        }

        return controller_interface::return_type::OK;
    }

protected:
    enum WheelLocation {
        FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT,
    };

    struct WheelVelocity {
        double front_left = 0;
        double front_right = 0;
        double rear_left = 0;
        double rear_right = 0;
    };

    std::shared_ptr<mechanum_controller::ParamListener> param_listener;
    mechanum_controller::Params params;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_odom_pub;
    std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> odom_pub_rt;
    std::unique_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> tf_odom_pub_rt;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_teleop = nullptr;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_nav = nullptr;
    realtime_tools::RealtimeBuffer<geometry_msgs::msg::TwistStamped> vel_msg;

    rclcpp::Duration cmd_vel_timeout = rclcpp::Duration(0,500000000);    // 受信したcmd_velが有効とみなす時間のタイムアウト
    rclcpp::Duration teleop_sub_timeout = rclcpp::Duration(1,0); // teleopの受信タイムアウト

    rclcpp::Time last_teleop_sub_time;
    std::mutex cmdvel_mtx;

    WheelVelocity cmd_1stlast; // last
    WheelVelocity cmd_2ndlast; // 2nd to last
    SpeedLimiter limiter_front_left;
    SpeedLimiter limiter_front_right;
    SpeedLimiter limiter_rear_left;
    SpeedLimiter limiter_rear_right;

    mechanum_controller::Velocity tgt_vel_1stlast;
    mechanum_controller::Velocity tgt_vel_2ndlast;
    SpeedLimiter limiter_x;
    SpeedLimiter limiter_y;
    SpeedLimiter limiter_th;

    Odometry odom;
    Kinematics kinema;
};

}
