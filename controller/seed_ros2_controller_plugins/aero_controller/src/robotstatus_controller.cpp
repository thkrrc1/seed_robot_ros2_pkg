#include <pluginlib/class_list_macros.hpp>
#include "robotstatus_controller.hpp"

#include "robotstatus.hpp"

namespace aero_controller {

RobotStatusController::RobotStatusController(){
    periodic_thread = std::thread(&RobotStatusController::periodic_send,this);
};

RobotStatusController::~RobotStatusController(){
    shutdown.store(true);
    if (periodic_thread.joinable()) {
        periodic_thread.join();
    }
}

controller_interface::CallbackReturn RobotStatusController::on_configure(const rclcpp_lifecycle::State &previous_state) {
    status_pub = get_node()->create_publisher<aero_controller_msgs::msg::RobotStatus>(std::string(get_node()->get_name()) + "/robotstatus",1);
    status_parsed_pub = get_node()->create_publisher<aero_controller_msgs::msg::RobotStatusParsedList>(std::string(get_node()->get_name()) + "/robotstatus_parsed",1);
    return controller_interface::CallbackReturn::SUCCESS;
}

void RobotStatusController::periodic_send(){
    while (!shutdown.load()) {

        auto mslist = getMsList();
        for (int &ms : mslist) {

            BuffRaw buff;
            makeBuffRobotStatus(getProtocol(ms), buff, ms);
            addSendData(ms, buff);
            //std::cout << "msid ID : " << ms << std::endl;
        }

        //100[ms]ごとに要求を投げる
        usleep(100000);
    }
}

void RobotStatusController::execute(const std::string &protocol, const BuffRaw *buf_recv, BuffRaw *buf_send) {
    //checksumの確認は実施済みなので、ここでは実行しない。

    if (getDataRobotStatus(protocol, *buf_recv, status_msg)) {
        status_pub->publish(status_msg);
        RobotStatusController::parse_status(status_msg);
        status_parsed_pub->publish(status_parsed_msg);
    }
}

void RobotStatusController::parse_status(const aero_controller_msgs::msg::RobotStatus &status){
    // any初期化
    status_parsed_msg.any_connection = 0;
    status_parsed_msg.any_org = 0;
    status_parsed_msg.any_moter_state = 0;
    status_parsed_msg.any_temperature_state = 0;
    status_parsed_msg.any_response = 0;
    status_parsed_msg.any_step_out = 0;
    status_parsed_msg.any_protection_stop = 0;
    status_parsed_msg.any_power_failure = 0;

    std::size_t i = 0;
    for(aero_controller_msgs::msg::RobotStatusParsed& status_parsed : status_parsed_msg.status_parsed_list){
        status_parsed.aero_idx = i;
        if (std::find(active_aero_idx_list.begin(), active_aero_idx_list.end(), status_parsed.aero_idx) != active_aero_idx_list.end()) {
            std::bitset<8> bs(status.data[i*2+1]); //2byteの下位1byteのみ使用
            status_parsed.active = 1;
            status_parsed.connection = bs[0];
            status_parsed.org = bs[1];
            status_parsed.moter_state = bs[2];
            status_parsed.temperature_state = bs[3];
            status_parsed.response = bs[4];
            status_parsed.step_out = bs[5];
            status_parsed.protection_stop = bs[6];
            status_parsed.power_failure = bs[7];
        }
        else{
            status_parsed.active = 0;
        }
        // any更新
        if( status_parsed.active == 1){
            status_parsed_msg.any_connection = status_parsed.connection | status_parsed_msg.any_connection;
            status_parsed_msg.any_org = status_parsed.org | status_parsed_msg.any_org;
            status_parsed_msg.any_moter_state = status_parsed.moter_state | status_parsed_msg.any_moter_state;
            status_parsed_msg.any_temperature_state = status_parsed.temperature_state | status_parsed_msg.any_temperature_state;
            status_parsed_msg.any_response = status_parsed.response | status_parsed_msg.any_response;
            status_parsed_msg.any_step_out = status_parsed.step_out | status_parsed_msg.any_step_out;
            status_parsed_msg.any_protection_stop = status_parsed.protection_stop | status_parsed_msg.any_protection_stop;
            status_parsed_msg.any_power_failure = status_parsed.power_failure | status_parsed_msg.any_power_failure;
        }
        i++;
    }
}

}

PLUGINLIB_EXPORT_CLASS(aero_controller::RobotStatusController, controller_interface::ControllerInterface)
