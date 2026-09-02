#include <pluginlib/class_list_macros.hpp>
#include "current_controller.hpp"
#include <rt_logger/logger.hpp>

namespace aero_controller {

CurrentController::CurrentController() : converter_loader_("seed_ros2_controller", "seed_converter::StrokeConverter"){
}

CurrentController::~CurrentController(){
    stop_periodic_thread();
}

controller_interface::CallbackReturn CurrentController::on_init() {
    // 位置送信処理暫定対応
    cur_angles.resize(30, 0.0);
    update_rate_default = 100;
    convert_lib_name = "seed_converter/NoidLifterMover";

    // 自分自身の controller node parameter から読む
    try {
        auto node = get_node();

        if (!node->has_parameter("converter_lib")) {
            node->declare_parameter<std::string>(
                "converter_lib",
                "seed_converter/NoidLifterMover");
        }

        if (!node->has_parameter("update_rate")) {
            node->declare_parameter<int>("update_rate", 100);
        }

        node->get_parameter("converter_lib", convert_lib_name);
        node->get_parameter("update_rate", update_rate_default);

    } catch (const std::exception &e) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Failed to read CurrentController parameters: %s",
            e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    shutdown.store(false);
    current_request_inflight.store(false);
    periodic_thread =std::thread(&CurrentController::periodic_send,this);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CurrentController::on_configure(const rclcpp_lifecycle::State &previous_state) {
    auto current_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    current_pub = get_node()->create_publisher<aero_controller_msgs::msg::Current>(std::string(get_node()->get_name()) + "/current_raw", current_qos);
    current_parsed_list_pub = get_node()->create_publisher<aero_controller_msgs::msg::CurrentParsedList>(std::string(get_node()->get_name()) + "/current_parsed",1);

    // 位置送信処理暫定対応
    jstate_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::ConstSharedPtr joint_state)
        {

            std::lock_guard<std::mutex> lock(jstate_mtx_);
            jstate.name.clear();
            jstate.position.clear();
            jstate = *joint_state;
            has_joint_state_ = true;
        });

    try {
        stroke_converter_ = converter_loader_.createSharedInstance(convert_lib_name);
        stroke_converter_->initialize();
    } catch (pluginlib::PluginlibException &ex) {
        LOG_ERROR_STREAM() << "The plugin failed to load : " << ex.what() << LOG_END;
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

void CurrentController::stop_periodic_thread() {
    shutdown.store(true);
    current_request_inflight.store(false);
    if (periodic_thread.joinable()) {
        periodic_thread.join();
    }
}

void CurrentController::periodic_send() {
    constexpr auto RESPONSE_TIMEOUT =std::chrono::milliseconds(10); //controller_manager: 1/update_rate [ms] 以上の値にする。
    while (!shutdown.load()) {
        auto now = std::chrono::steady_clock::now();
        if (current_request_inflight.load()) {
            auto elapsed = duration_cast<std::chrono::milliseconds>(now - last_send_time);
            if (elapsed > RESPONSE_TIMEOUT) {
                current_request_inflight.store(false);
            }
        }

        if (!current_request_inflight.load()) {
            makeBuffCurrent(getProtocol(ms), current_buff, ms);
            bool ok_send = addSendData(ms, current_buff);
            if (ok_send) {
                last_send_time = std::chrono::steady_clock::now();
                current_request_inflight.store(true);
            }
        }
        usleep(10000);
    }
}

void CurrentController::execute(const std::string &protocol, const BuffRaw *buf_recv, BuffRaw *buf_send) {
    //checksumの確認は実施済みなので、ここでは実行しない。

    if (getDataCurrent(protocol, *buf_recv, current_msg)) {

        // 位置送信処理暫定対応
        {
            std::lock_guard<std::mutex> lock(jstate_mtx_);

            if (has_joint_state_) {
                jnames = getJointNames(ms);
                for (size_t idx = 0; idx < jstate.name.size(); ++idx) {
                    auto itr = std::find(jnames.begin(), jnames.end(), jstate.name[idx]);
                    if (itr != jnames.end()) {
                        int send_idx = std::distance(jnames.begin(), itr);
                        cur_angles[send_idx] = jstate.position[idx];
                    }
                }
            }
        }
        if (stroke_converter_ && !cur_angles.empty()) {
            stroke_converter_->setJointNames(jnames);
            std::vector<int16_t> strokes;
            strokes.resize(cur_angles.size(), 0x7FFF);
            stroke_converter_->Angle2Stroke(strokes, cur_angles);
            const size_t n = std::min(current_msg.pos_data.size(),strokes.size());
            for (size_t idx = 0; idx < n; ++idx) {
                current_msg.pos_data[idx] = strokes[idx];
            }
        }

        current_request_inflight.store(false);
        current_pub->publish(current_msg);
        CurrentController::parse_current(current_msg);
        current_parsed_list_pub->publish(current_parsed_list_msg);
    }
}

void CurrentController::parse_current(const aero_controller_msgs::msg::Current &current){
    std::size_t i = 0;
    for(aero_controller_msgs::msg::CurrentParsed& current_parsed : current_parsed_list_msg.current_parsed_list){
        if(i*2+1 >= current.data.size()){
            break;
        }
        // 電流(Motor)
        std::bitset<16> bs2;
        float decimal_value;
        bs2 = std::bitset<16>((current.data[i*2] << 8) | current.data[i*2+1]);
        decimal_value = static_cast<uint16_t>(bs2.to_ulong());
        current_parsed.current = decimal_value;
        current_parsed.aero_idx = i;
        i++;
    }
}

}

PLUGINLIB_EXPORT_CLASS(aero_controller::CurrentController, controller_interface::ControllerInterface)
