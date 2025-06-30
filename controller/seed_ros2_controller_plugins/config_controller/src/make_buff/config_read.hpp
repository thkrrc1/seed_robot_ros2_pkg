#pragma once

#include <seed_ros2_controller/command_interface/other_command/other_cmd_buff.hpp>
#include <seed_ros2_controller/aero/version3/send_buff_aero3.hpp>
#include <seed_ros2_controller/aero/version4/send_buff_aero4.hpp>
#include <config_controller_msgs/srv/ms_config_read.hpp>

namespace config_controller {

template<class T>
bool makeBuffImpl(BuffRaw &buff, int msid, config_controller_msgs::srv::MsConfigRead::Request &req) {
    return false;
}

template<>
bool makeBuffImpl<aero3::SendBuff>(BuffRaw &buff, int msid, config_controller_msgs::srv::MsConfigRead::Request &req) {
    buff.clear();
    aero3::SendBuff *reqdata = reinterpret_cast<aero3::SendBuff*>(buff.data);
    reqdata->init();
    reqdata->header.data[0] = 0xFC;
    reqdata->header.data[1] = 0xCF;
    reqdata->cmd = 0x00;

    int cur_idx = 0;
    uint8_t *addr = reinterpret_cast<uint8_t*>(&req.address);
    uint8_t size = req.size;
    reqdata->mcid = addr[1];
    cur_idx = reqdata->setData(addr[0], cur_idx);
    cur_idx = reqdata->setData(size, cur_idx);
    reqdata->addChecksum(); //ここはチェックサム必要

    buff.size = reqdata->getTotalLen();

    return true;
}

template<>
bool makeBuffImpl<aero4::SendBuff>(BuffRaw &buff, int msid, config_controller_msgs::srv::MsConfigRead::Request &req) {
    buff.clear();
    aero4::SendBuff *reqdata = reinterpret_cast<aero4::SendBuff*>(buff.data);
    reqdata->init();
    reqdata->header.data[0] = 0xFC;
    reqdata->header.data[1] = 0xCF;
    reqdata->cmd = 0x00;
    reqdata->msid = msid;

    int cur_idx = 0;
    uint16_t addr = req.address;
    cur_idx = reqdata->setData(addr, cur_idx);
    reqdata->addChecksum();
    buff.size = reqdata->getTotalLen();

    return true;
}

bool makeBuff(const std::string &protocol, BuffRaw &buff, int msid, config_controller_msgs::srv::MsConfigRead::Request &req) {
    if (protocol == "aero3") {
        return makeBuffImpl<aero3::SendBuff>(buff, msid, req);
    } else {
        return makeBuffImpl<aero4::SendBuff>(buff, msid, req);
    }
}


}
