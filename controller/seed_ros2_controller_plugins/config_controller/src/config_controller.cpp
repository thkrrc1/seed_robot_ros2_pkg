#include "config_controller.hpp"

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <seed_ros2_controller/aero/version3/recv_buff_aero3.hpp>
#include <seed_ros2_controller/aero/version4/recv_buff_aero4.hpp>

#include "config_read.hpp"
#include "config_write.hpp"
#include "ms_reset.hpp"

#include "rclcpp/qos.hpp"

#include <rt_logger/logger.hpp>

namespace config_controller {


template<class T>
bool getDataImpl(const BuffRaw &buff, std::vector<uint8_t>& out){
    return false;
}

template<>
bool getDataImpl<aero3::RecvBuff>(const BuffRaw &buff, std::vector<uint8_t>& out){
    auto *respd = reinterpret_cast<const aero3::RecvBuff*>(buff.data);
    if (respd->cmd != 0x00 || respd->datalen() <= 0) {
        LOG_ERROR_STREAM() << "config command response error cmd:" << (int) respd->cmd << " len:" << (int) respd->datalen() << LOG_END;
        return false;
    }

    out = std::vector < uint8_t > (std::begin(respd->data)+1, std::begin(respd->data) + respd->datalen());
    return true;
}

template<>
bool getDataImpl<aero4::RecvBuff>(const BuffRaw &buff, std::vector<uint8_t>& out){
    auto *respd = reinterpret_cast<const aero4::RecvBuff*>(buff.data);
    if (respd->cmd != 0x00) {
        LOG_ERROR_STREAM() << "config command response error cmd:" << (int) respd->cmd << " len:" << (int) respd->datalen() << LOG_END;
        return false;
    }

    out = std::vector < uint8_t > (std::begin(respd->data)+2, std::end(respd->data));
    return true;
}


bool getData(const std::string &protocol, const BuffRaw &buff, std::vector<uint8_t> &out) {
    if (protocol == "aero4") {
        return getDataImpl < aero4::RecvBuff > (buff, out);
    } else {
        return getDataImpl < aero3::RecvBuff > (buff, out);
    }
}


ConfigController::ConfigController() {
}

controller_interface::CallbackReturn ConfigController::on_configure(const rclcpp_lifecycle::State &previous_state) {

    using namespace std::placeholders;
    config_begin_srv_ = get_node()->create_service < std_srvs::srv::Empty >(std::string(get_node()->get_name()) + "/config_begin", std::bind(&ConfigController::configBegin, this, _1, _2));
    config_end_srv_ = get_node()->create_service < std_srvs::srv::Empty >(std::string(get_node()->get_name()) + "/config_end", std::bind(&ConfigController::configEnd, this, _1, _2));
    config_write_srv_ = get_node()->create_service < config_controller_msgs::srv::MsConfigWrite >(std::string(get_node()->get_name()) + "/config_write", std::bind(&ConfigController::configWrite, this, _1, _2));
    config_read_srv_ = get_node()->create_service < config_controller_msgs::srv::MsConfigRead >(std::string(get_node()->get_name()) + "/config_read", std::bind(&ConfigController::configRead, this, _1, _2));

    grp_ = get_node()->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_services_default));

    switch_client_ = get_node()->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller", qos_profile, grp_);

    list_client_ = get_node()->create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers", qos_profile, grp_);

//    client_othercmd_only = get_node()->serviceClient < std_srvs::SetBool > ("/seed_ros_controller/otherCmdOnly");
    return controller_interface::CallbackReturn::SUCCESS;
}

bool ConfigController::configBegin(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) {

    //Otherコマンド以外を受け付けないようにする
//    std_srvs::SetBool msg_othercmd_only;
//    msg_othercmd_only.request.data = true;
//    if (!client_othercmd_only.call(msg_othercmd_only)) {
//        return false;
//    }
//

    using namespace std::chrono_literals;
    auto list_req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    if (!list_client_->wait_for_service(1s)) {
        return false;
    }
    //自分以外のコントローラを停止させる
    using namespace std::placeholders;
    auto future = list_client_->async_send_request(list_req);
    auto controllers = future.get()->controller;
    for (auto &controller : controllers) {
        if (controller.state == "active" && controller.name != "config_controller") {
            halted_controllers_.push_back(controller.name);
        }
    }

    if (halted_controllers_.size() != 0) {
        auto switch_req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        switch_req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
        switch_req->activate_asap = false; //全コントローラが切り替わってから、update処理を呼び始める
        switch_req->timeout = rclcpp::Duration(1.0s);
        switch_req->deactivate_controllers = halted_controllers_;
        if (!switch_client_->wait_for_service(1s)) {
            return false;
        }
        auto future = switch_client_->async_send_request(switch_req);
        future.get();
    }

    ms_need_reset.clear();
    return true;
}

bool ConfigController::configEnd(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) {


    for (auto msid : ms_need_reset) {
        BuffRaw buff;
        makeBuff(getProtocol(msid), buff, msid);
        addSendData(msid, buff);
        //RESETの応答はなさそうなので、何もしない。
    }

    //ロボットの再起動が必要な場合は、コマンドの受付を再開しない。
    if (ms_need_reset.size() != 0) {
        return true;
    }

    //全コマンドを受け付けられるようにする
//    std_srvs::SetBool msg_othercmd_only;
//    msg_othercmd_only.request.data = false;
//    if (!client_othercmd_only.call(msg_othercmd_only)) {
//        return false;
//    }
//
//


    //コントローラーを再開する
    if (halted_controllers_.size() != 0) {
        using namespace std::chrono_literals;
        auto switch_req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        switch_req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
        switch_req->activate_asap = false; //全コントローラが切り替わってから、update処理を呼び始める
        switch_req->timeout = rclcpp::Duration(1.0s);
        switch_req->activate_controllers = halted_controllers_;
        if (!switch_client_->wait_for_service(1s)) {
            return false;
        }

        auto future = switch_client_->async_send_request(switch_req);
        future.get();
        halted_controllers_.clear();
    }

    return true;
}

bool checkDataCommand(const BuffRaw &buff, uint8_t cmd) {
    if (buff.data[3] == cmd) {
        return true;
    }
    return false;
}


bool ConfigController::configWrite(std::shared_ptr<config_controller_msgs::srv::MsConfigWrite::Request> req, std::shared_ptr<config_controller_msgs::srv::MsConfigWrite::Response> res) {

    BuffRaw buff;
    //まず最初に、現在設定を読み込む
    config_controller_msgs::srv::MsConfigRead::Request req_read;
    config_controller_msgs::srv::MsConfigRead::Response res_read;
    req_read.msid = req->msid;
    req_read.address = req->address;
    req_read.size = req->data.size();
    makeBuff(getProtocol(req_read.msid), buff, req_read.msid, req_read);


    //応答を取得
    std::unique_lock < std::mutex > lk(mtx);
    addSendData(req_read.msid, buff);
    std::cv_status cond_result = cond.wait_for(lk, std::chrono::seconds(2));
    if (cond_result == std::cv_status::timeout) {
        LOG_ERROR_STREAM() << "config command response timeout" << LOG_END;
        return false;
    }

    if(!getData(getProtocol(req_read.msid),recv_buff_raw, res_read.data)){
        return false;
    }

    //送信値との差分を取る
    bool need_send = !std::equal(req->data.cbegin(), req->data.cend(), res_read.data.cbegin());
    if (!need_send) {
        LOG_INFO_STREAM() << "SKIP : There are no changes on ROM data at address : "<<req->address<<LOG_END;
        return true;
    }

    //送信データを現在のデータとマージする。
    if (req->data.size() < res_read.data.size()) {
        req->data.reserve(res_read.data.size());
        std::copy(res_read.data.begin() + req->data.size(), res_read.data.end(), std::back_inserter(req->data));
    }

    makeBuff(getProtocol(req->msid), buff, req->msid, *req);
    addSendData(req->msid, buff);

    cond_result = cond.wait_for(lk, std::chrono::seconds(2)); //応答までに1秒ちょっとかかるので、少し待つ
    if (cond_result == std::cv_status::timeout) {
        LOG_ERROR_STREAM() << "config command response timeout" << LOG_END;
        return false;
    }

    if (std::find(ms_need_reset.begin(), ms_need_reset.end(), req->msid) == ms_need_reset.end()) {
        ms_need_reset.push_back(req->msid); // 書き込んだデータを反映させるには、リセットが必要？
    }

    //受信データのコマンド部がFFであれば、書き込みに対する応答
    return checkDataCommand(recv_buff_raw, 0xFF);
}



bool ConfigController::configRead(std::shared_ptr<config_controller_msgs::srv::MsConfigRead::Request> req, std::shared_ptr<config_controller_msgs::srv::MsConfigRead::Response> res) {
    BuffRaw buff;
    makeBuff(getProtocol(req->msid), buff, req->msid, *req);

    std::unique_lock < std::mutex > lk(mtx);
    addSendData(req->msid, buff);
    std::cv_status result = cond.wait_for(lk, std::chrono::milliseconds(500));
    if (result == std::cv_status::timeout) {
        LOG_ERROR_STREAM() << "config command response timeout" << LOG_END;
        return false;
    }

    return getData(getProtocol(req->msid),recv_buff_raw, res->data);
}

void ConfigController::execute(const std::string& protocol, const BuffRaw *buf_recv, BuffRaw *buf_send) {
    std::lock_guard < std::mutex > lk(mtx);

    recv_buff_raw = *buf_recv;

    cond.notify_one();
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(config_controller::ConfigController, controller_interface::ControllerInterface)
