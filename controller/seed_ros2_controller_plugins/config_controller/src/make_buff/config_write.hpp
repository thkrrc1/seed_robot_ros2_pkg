#pragma once

#include <seed_ros2_controller/command_interface/other_command/other_cmd_buff.hpp>
#include <seed_ros2_controller/aero/version3/send_buff_aero3.hpp>
#include <seed_ros2_controller/aero/version4/send_buff_aero4.hpp>
#include <config_controller_msgs/srv/ms_config_write.hpp>

namespace config_controller {

template<class T>
void makeBuffImpl(BuffRaw &buff, int msid, config_controller_msgs::srv::MsConfigWrite::Request &req) {
}

template<>
void makeBuffImpl<aero3::SendBuff>(BuffRaw &buff, int msid, config_controller_msgs::srv::MsConfigWrite::Request &req) {
    buff.clear();
    aero3::SendBuff *reqdata = reinterpret_cast<aero3::SendBuff*>(buff.data);
    reqdata->init();
    reqdata->header.data[0] = 0xFC;
    reqdata->header.data[1] = 0xCF;
    reqdata->cmd = 0xFF;

    int cur_idx = 0;
    uint8_t *addr = reinterpret_cast<uint8_t*>(&req.address);
    reqdata->mcid = addr[1];
    cur_idx = reqdata->setData(addr[0], cur_idx);
    cur_idx = reqdata->setData(req.data.data(), req.data.size(), cur_idx);
    reqdata->len -= 1;//チェックサムは不要なので、1バイト分引いておく
    buff.size = reqdata->getTotalLen();
}

template<>
void makeBuffImpl<aero4::SendBuff>(BuffRaw &buff, int msid, config_controller_msgs::srv::MsConfigWrite::Request &req) {
    buff.clear();
    aero4::SendBuff *reqdata = reinterpret_cast<aero4::SendBuff*>(buff.data);
    reqdata->init();
    reqdata->header.data[0] = 0xFC;
    reqdata->header.data[1] = 0xCF;
    reqdata->msid = msid;
    reqdata->cmd = 0xFF;

    int cur_idx = 0;
    cur_idx = reqdata->setData(req.address, cur_idx);
    cur_idx = reqdata->setData(req.data.data(), req.data.size(), cur_idx);
    reqdata->addChecksum();
    buff.size = reqdata->getTotalLen();
}

void makeBuff(const std::string &protocol, BuffRaw &buff, int msid, config_controller_msgs::srv::MsConfigWrite::Request &req) {
    if (protocol == "aero3") {
        makeBuffImpl<aero3::SendBuff>(buff, msid, req);
    } else {
        makeBuffImpl<aero4::SendBuff>(buff, msid, req);
    }
}

}
