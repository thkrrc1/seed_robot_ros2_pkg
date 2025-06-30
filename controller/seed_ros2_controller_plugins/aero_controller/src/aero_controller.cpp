#include <chrono>
#include <rt_logger/logger.hpp>
#include "aero_controller.hpp"

#include "servo.hpp"
#include "set_current.hpp"
#include "run_script.hpp"
#include "get_dio.hpp"
#include "get_version.hpp"
#include "get_ad.hpp"
#include "can_through.hpp"


namespace aero_controller {


AeroController::AeroController() {
    cmd_thread = std::thread(&AeroController::cmdSendThread,this);
}

AeroController::~AeroController() {
    running.store(false);
    if (cmd_thread.joinable()) {
        cmd_thread.join();
    }
}


controller_interface::CallbackReturn AeroController::on_configure(const rclcpp_lifecycle::State &previous_state) {
    using namespace std::placeholders;
    servo_srv = get_node()->create_service < aero_controller_msgs::srv::Servo >(std::string(get_node()->get_name()) + "/servo",  std::bind(&AeroController::servo, this, _1, _2));
    scur_srv = get_node()->create_service < aero_controller_msgs::srv::SetCurrent >(std::string(get_node()->get_name()) + "/set_current",  std::bind(&AeroController::setCurrent, this, _1, _2));
    rscr_srv = get_node()->create_service < aero_controller_msgs::srv::RunScript >(std::string(get_node()->get_name()) + "/run_script",  std::bind(&AeroController::runScript, this, _1, _2));
    verget_srv = get_node()->create_service < aero_controller_msgs::srv::GetVersion >(std::string(get_node()->get_name()) + "/get_version",  std::bind(&AeroController::getVersion, this, _1, _2));
    adget_srv = get_node()->create_service < aero_controller_msgs::srv::GetAd >(std::string(get_node()->get_name()) + "/get_ad",  std::bind(&AeroController::getAd, this, _1, _2));
    canthrough_srv = get_node()->create_service < aero_controller_msgs::srv::CanThrough >(std::string(get_node()->get_name()) + "/can_through",  std::bind(&AeroController::canThrough, this, _1, _2)); 
    return controller_interface::CallbackReturn::SUCCESS;
}

bool AeroController::servo(std::shared_ptr<aero_controller_msgs::srv::Servo::Request> req, std::shared_ptr<aero_controller_msgs::srv::Servo::Response> res) {
    std::vector<int> mslist;
    if (req->msid == 0) { // msidが0の場合は全MS
        mslist = getMsList();
    }else{
        mslist = std::vector<int>({req->msid});
    }

    BuffRaw buff;
    for (int &ms : mslist) {
        makeBuff(getProtocol(ms), buff, ms, *req);
        addSendData(ms, buff);
    }
    return true;
}

bool AeroController::setCurrent(std::shared_ptr<aero_controller_msgs::srv::SetCurrent::Request> req, std::shared_ptr<aero_controller_msgs::srv::SetCurrent::Response> res) {

    auto mslist = getMsList();
    for (auto &ms : mslist) {
        BuffRaw buff;
        if (makeBuff(getProtocol(ms), buff, ms, getJointNames(ms), *req)) {
            addSendData(ms, buff);
        }
    }

    usleep(100000);//応答が返ってこない&短い周期で送信するとMSが受け付けないので、確実に実行されるまで待つ

    return true;
}

bool AeroController::runScript(std::shared_ptr<aero_controller_msgs::srv::RunScript::Request> req, std::shared_ptr<aero_controller_msgs::srv::RunScript::Response> res) {
    auto script_context = createScriptContext(*req);
    bool ret = scr_ctrl.runScript(this, script_context);
    if (ret) {
        *res = createScriptResponse(script_context);
    }
    return ret;
}

bool AeroController::getVersion(std::shared_ptr<aero_controller_msgs::srv::GetVersion::Request> req, std::shared_ptr<aero_controller_msgs::srv::GetVersion::Response> res) {
    BuffRaw buff;

    auto mslist = getMsList();
    for (auto &ms : mslist) {
        auto protocol = getProtocol(ms);
        int mcmax = 27;
        if (protocol == "aero3") {
            mcmax = 30;
        }

        // MSのバージョン取得
        aero_controller_msgs::msg::VersionMs verInfo;
        verInfo.msid = ms;
        verInfo.version = verget_ctrl.getVersion(this, ms);

        //MCのバージョン取得
        for (int mc = 1; mc < mcmax; ++mc) {
            verInfo.version_mc.push_back(verget_ctrl.getVersion(this, ms, mc));
        }
        res->versions.push_back(verInfo);
    }

    return true;
}

bool AeroController::getAd(std::shared_ptr<aero_controller_msgs::srv::GetAd::Request> req, std::shared_ptr<aero_controller_msgs::srv::GetAd::Response> res) {
    std::vector<int> value_data;
    fill(value_data.begin(), value_data.end(), 0);
    // ad_noに応じてad値1~30まで取得
    adget_ctrl.getAd(this, req->ad_no, value_data);
    for(int data : value_data){
        res->value.push_back(data);
    }

    return true;
}

bool AeroController::canThrough(std::shared_ptr<aero_controller_msgs::srv::CanThrough::Request> req, std::shared_ptr<aero_controller_msgs::srv::CanThrough::Response> res) {
    int msid = 0;
    msid = req->msid;

    BuffRaw buff;
    makeBuff(getProtocol(msid), buff, msid, *req);
    addSendData(msid, buff);

    return true;
}

void AeroController::execute(int msid, const std::string &protocol, const BuffRaw *buf_recv, BuffRaw *buf_send) {
    scr_ctrl.setRecvData(msid, protocol, buf_recv, buf_send);
    verget_ctrl.setRecvData(msid, protocol, buf_recv, buf_send);
    adget_ctrl.setRecvData(msid, protocol, buf_recv, buf_send);
}

aero_controller_msgs::srv::RunScript::Request AeroController::createScriptRequest(const ScriptContext &context) const {
    aero_controller_msgs::srv::RunScript::Request ret;
    for (auto& [msid, ms_context] : context.ms_contexts) {
        for (auto& [mcid, mc_context] : ms_context.mc_contexts) {
            aero_controller_msgs::msg::ScriptReqJNoInterf jreq;
            jreq.msid = msid;
            jreq.send_no = mcid;
            jreq.script_no = mc_context.script_no;
            jreq.arg = mc_context.arg;
            ret.jno_interf.push_back(jreq);
        }
    }
    return ret;
}

aero_controller_msgs::srv::RunScript::Response AeroController::createScriptResponse(const ScriptContext &context) const {
    aero_controller_msgs::srv::RunScript::Response ret;
    for (auto& [msid, ms_context] : context.ms_contexts) {
        auto jnames = getJointNames(msid);
        for (auto& [mcid, mc_context] : ms_context.mc_contexts) {
            aero_controller_msgs::msg::ScriptResJoint jres;
            jres.msid = msid;
            jres.send_no = mcid;
            jres.joint_name = jnames[mcid];
            jres.result = mc_context.cur_dio;
            ret.jres.push_back(jres);
        }
    }
    return ret;
}

ScriptContext AeroController::createScriptContext(const aero_controller_msgs::srv::RunScript::Request &req) const {
    ScriptContext ret;
    ret.timeout_sec = req.timeout_sec;
    auto mslist = getMsList();
    for (auto &msid : mslist) {
        ScriptContextMs ms_context;

        auto jnames = getJointNames(msid);
        for (auto &request : req.jname_interf) {
            auto itr = std::find(jnames.begin(), jnames.end(), request.joint_name);
            if (itr != jnames.end()) {
                int send_idx = std::distance(jnames.begin(), itr);
                ScriptContextMc mc_context;
                mc_context.script_no = request.script_no;
                mc_context.arg = request.arg;
                mc_context.run_dio = request.dio_run;
                ms_context.mc_contexts[send_idx] = mc_context;
            }
        }

        //関節番号指定の展開
        for (auto &request : req.jno_interf) {
            if (request.msid == msid) {
                int send_idx = request.send_no;
                ScriptContextMc mc_context;
                mc_context.script_no = request.script_no;
                mc_context.arg = request.arg;
                mc_context.run_dio = request.dio_run;
                ms_context.mc_contexts[send_idx] = mc_context;
            }
        }

        ret.ms_contexts[msid] = ms_context;
    }

    return ret;
}

void AeroController::sendScriptCommand(const ScriptContext &context) {
    auto req = createScriptRequest(context);
    auto mslist = getMsList();
    for (auto &ms : mslist) {
        BuffRaw buff;
        if (makeBuff(getProtocol(ms), buff, ms, getJointNames(ms), req)) {
            addSendData(ms, buff);
        }
    }
}

void AeroController::sendDioCommand() {
    std::lock_guard < std::mutex > lk(cmd_mtx);
    cmd_req = true;
}

void AeroController::sendVerGetCommand(uint8_t msid, uint8_t mcid) {
    BuffRaw buff;
    if (makeBuffGetVersion(getProtocol(msid), buff, msid, mcid)) {
        addSendData(msid, buff);
    }
}

void AeroController::sendAdGetCommand(int ad_no){
    BuffRaw buff;
    int msid = 1;
    if (makeBuffGetAd(getProtocol(msid), buff, msid, ad_no)) {
        if(!addSendData(msid, buff)){
            LOG_ERROR_STREAM()<< "Failed to send Ad retrieval command" << LOG_END;
        }
    }
}

void AeroController::cmdSendThread() {
    while (running.load()) {
        usleep(10000); // 要求をまとめて処理する
        std::unique_lock < std::mutex > lk(cmd_mtx);
        if (!cmd_req) {
            continue;
        }
        cmd_req = false;
        lk.unlock();

        auto mslist = getMsList();
        for (auto &ms : mslist) {
            BuffRaw buff;
            if (makeBuffGetDio(getProtocol(ms), buff, ms)) {
                addSendData(ms, buff);
            }
        }
    }
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(aero_controller::AeroController, controller_interface::ControllerInterface)
