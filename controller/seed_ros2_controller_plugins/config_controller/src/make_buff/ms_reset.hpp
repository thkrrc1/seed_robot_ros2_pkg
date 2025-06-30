#pragma once

#include <seed_ros2_controller/command_interface/other_command/other_cmd_buff.hpp>
#include <seed_ros2_controller/aero/version3/send_buff_aero3.hpp>
#include <seed_ros2_controller/aero/version4/send_buff_aero4.hpp>

namespace config_controller {

template<class T>
bool makeBuffImpl(BuffRaw &buff, int msid) {
    return false;
}

template<>
bool makeBuffImpl<aero3::SendBuff>(BuffRaw &buff, int msid) {
    buff.clear();
    aero3::SendBuff *reqdata = reinterpret_cast<aero3::SendBuff*>(buff.data);
    reqdata->init();
    reqdata->header.data[0] = 0xFC;
    reqdata->header.data[1] = 0xCF;
    reqdata->cmd = 0x7F;
    reqdata->mcid = 0x00;
    reqdata->addChecksum();
    buff.size = reqdata->getTotalLen();
    return true;
}

template<>
bool makeBuffImpl<aero4::SendBuff>(BuffRaw &buff, int msid) {
    buff.clear();
    aero4::SendBuff *reqdata = reinterpret_cast<aero4::SendBuff*>(buff.data);
    reqdata->init();
    reqdata->header.data[0] = 0xFC;
    reqdata->header.data[1] = 0xCF;
    reqdata->cmd = 0x7F;
    reqdata->msid = msid;
    reqdata->addChecksum();
    buff.size = reqdata->getTotalLen();
    return true;
}

bool makeBuff(const std::string &protocol, BuffRaw &buff, int msid) {
    if (protocol == "aero3") {
        return makeBuffImpl<aero3::SendBuff>(buff, msid);
    } else {
        return makeBuffImpl<aero4::SendBuff>(buff, msid);
    }
}
}
