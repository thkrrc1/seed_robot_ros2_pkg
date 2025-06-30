#pragma once

#include <seed_ros2_controller/command_interface/other_command/other_cmd_buff.hpp>

#include <aero_controller_msgs/srv/run_script.hpp>
#include <seed_ros2_controller/aero/version3/send_buff_aero3.hpp>
#include <seed_ros2_controller/aero/version4/send_buff_aero4.hpp>

namespace aero_controller {

template<class T>
bool makeBuffImpl(BuffRaw &buff, int msid, const std::vector<std::string> &jnames, aero_controller_msgs::srv::RunScript::Request &req) {
    return false;
}

template<>
bool makeBuffImpl<aero3::SendBuff>(BuffRaw &buff, int msid, const std::vector<std::string> &jnames, aero_controller_msgs::srv::RunScript::Request &req) {
    buff.clear();
    aero3::SendBuff *reqdata = reinterpret_cast<aero3::SendBuff*>(buff.data);
    reqdata->init();
    reqdata->header.data[0] = 0xFD;
    reqdata->header.data[1] = 0xDF;
    reqdata->cmd = 0x22;
    reqdata->mcid = 0x00;

    bool data_exist = false;
    //関節名指定の展開
    for (auto &request : req.jname_interf) {
        auto itr = std::find(jnames.begin(), jnames.end(), request.joint_name);
        if (itr != jnames.end()) {
            int send_idx = std::distance(jnames.begin(), itr) * 2; // 2バイトデータなので、2倍する
            reqdata->setData(request.arg, send_idx);
            reqdata->setData(request.script_no, send_idx + 1);
            data_exist = true;
        }
    }

    //関節番号指定の展開
    for (auto &request : req.jno_interf) {
        if (request.msid == msid) {
            int send_idx = request.send_no * 2; // 2バイトデータなので、2倍する
            reqdata->setData(request.arg, send_idx);
            reqdata->setData(request.script_no, send_idx + 1);
            data_exist = true;
        }
    }

    if (!data_exist) {
        return false;
    }

    reqdata->cram(); //データを送信する場合は、全軸分送る
    reqdata->addChecksum();

    buff.size = reqdata->getTotalLen();

    return true;
}

template<>
bool makeBuffImpl<aero4::SendBuff>(BuffRaw &buff, int msid, const std::vector<std::string> &jnames, aero_controller_msgs::srv::RunScript::Request &req) {
    buff.clear();
    aero4::SendBuff *reqdata = reinterpret_cast<aero4::SendBuff*>(buff.data);
    reqdata->init();
    reqdata->header.data[0] = 0xFD;
    reqdata->header.data[1] = 0xDF;
    reqdata->cmd = 0x22;
    reqdata->msid = msid;

    bool data_exist = false;
    //関節名指定の展開
    for (auto &request : req.jname_interf) {
        auto itr = std::find(jnames.begin(), jnames.end(), request.joint_name);
        if (itr != jnames.end()) {
            int send_idx = std::distance(jnames.begin(), itr) * 2; // 2バイトデータなので、2倍する
            reqdata->setData(request.script_no, send_idx); // 送信データ列は、Aero3と逆
            reqdata->setData(request.arg, send_idx + 1);
            data_exist = true;
        }
    }

    //関節番号指定の展開
    for (auto request : req.jno_interf) {
        if (request.msid == msid) {
            int send_idx = request.send_no * 2; // 2バイトデータなので、2倍する
            reqdata->setData(request.script_no, send_idx); // 送信データ列は、Aero3と逆
            reqdata->setData(request.arg, send_idx + 1);
            data_exist = true;
        }
    }

    if (!data_exist) {
        return false;
    }

    reqdata->addChecksum();
    buff.size = reqdata->getTotalLen();

    return true;
}

bool makeBuff(const std::string &protocol, BuffRaw &buff, int msid, const std::vector<std::string> &jnames, aero_controller_msgs::srv::RunScript::Request &req) {
    if (protocol == "aero3") {
        return makeBuffImpl<aero3::SendBuff>(buff, msid, jnames, req);
    } else {
        return makeBuffImpl<aero4::SendBuff>(buff, msid, jnames, req);
    }
}
}
