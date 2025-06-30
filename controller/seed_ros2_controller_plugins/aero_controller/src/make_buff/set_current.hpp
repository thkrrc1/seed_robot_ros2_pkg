#pragma once

#include <seed_ros2_controller/command_interface/other_command/other_cmd_buff.hpp>

#include "aero_controller_msgs/srv/set_current.hpp"
#include <seed_ros2_controller/aero/version3/send_buff_aero3.hpp>
#include <seed_ros2_controller/aero/version4/send_buff_aero4.hpp>

namespace aero_controller {

template<class T>
bool makeBuffImpl(BuffRaw &buff, int msid, const std::vector<std::string> &jnames, aero_controller_msgs::srv::SetCurrent::Request &req) {
    return false;
}

template<>
bool makeBuffImpl<aero3::SendBuff>(BuffRaw &buff, int msid, const std::vector<std::string> &jnames, aero_controller_msgs::srv::SetCurrent::Request &req) {
    buff.clear();
    aero3::SendBuff *reqdata = reinterpret_cast<aero3::SendBuff*>(buff.data);
    reqdata->init();
    reqdata->header.data[0] = 0xFD;
    reqdata->header.data[1] = 0xDF;
    reqdata->cmd = 0x01;
    reqdata->mcid = 0x00;

    bool data_exist = false;
    for (size_t idx = 0; idx < req.joint_name.size(); ++idx) {
        auto itr = std::find(jnames.begin(), jnames.end(), req.joint_name[idx]);
        if (itr != jnames.end() && idx < req.max.size() && idx < req.min.size()) {
            uint8_t max = req.max[idx];
            uint8_t min = req.min[idx];
            int send_idx = std::distance(jnames.begin(), itr);
            reqdata->setData(max, send_idx * 2);
            reqdata->setData(min, send_idx * 2 + 1);
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
bool makeBuffImpl<aero4::SendBuff>(BuffRaw &buff, int msid, const std::vector<std::string> &jnames, aero_controller_msgs::srv::SetCurrent::Request &req) {
    buff.clear();
    aero4::SendBuff *reqdata = reinterpret_cast<aero4::SendBuff*>(buff.data);
    reqdata->init();
    reqdata->header.data[0] = 0xFD;
    reqdata->header.data[1] = 0xDF;
    reqdata->cmd = 0x21;
    reqdata->msid = msid;

    bool data_exist = false;
    for (size_t idx = 0; idx < req.joint_name.size(); ++idx) {
        auto itr = std::find(jnames.begin(), jnames.end(), req.joint_name[idx]);
        if (itr != jnames.end() && idx < req.max.size() && idx < req.min.size()) {
            uint8_t max = req.max[idx];
            uint8_t min = req.min[idx];
            int send_idx = std::distance(jnames.begin(), itr);
            reqdata->setData(max, send_idx * 2 + 1); //リトルエンディアン
            reqdata->setData(min, send_idx * 2);
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

bool makeBuff(const std::string &protocol, BuffRaw &buff, int msid, const std::vector<std::string> &jnames, aero_controller_msgs::srv::SetCurrent::Request &req) {
    if (protocol == "aero3") {
        return makeBuffImpl<aero3::SendBuff>(buff, msid, jnames, req);
    } else {
        return makeBuffImpl<aero4::SendBuff>(buff, msid, jnames, req);
    }
}
}
