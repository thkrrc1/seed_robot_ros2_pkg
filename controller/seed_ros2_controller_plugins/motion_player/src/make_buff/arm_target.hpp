#pragma once

#include <seed_ros2_controller/command_interface/other_command/other_cmd_buff.hpp>

#include "arm_info.hpp"
#include <seed_ros2_controller/aero/version3/send_buff_aero3.hpp>
#include <seed_ros2_controller/aero/version4/send_buff_aero4.hpp>

namespace motion_player {

template<class T>
bool makeBuffImpl(BuffRaw &buff, int msid, const motion_player::ArmInfo &req, const int16_t strokes[], uint16_t duration) {
    return false;
}

template<>
bool makeBuffImpl<aero3::SendBuff>(BuffRaw &buff, int msid, const motion_player::ArmInfo &req, const int16_t strokes[], uint16_t duration) {
    aero3::SendBuff *reqdata = reinterpret_cast<aero3::SendBuff*>(buff.data);
    reqdata->init();
    reqdata->header.data[0] = 0xFD;
    reqdata->header.data[1] = 0xDF;
    reqdata->cmd = 0x14;
    reqdata->mcid = 0x00;

    for (size_t jidx = 0; jidx < req.jinfo.size(); ++jidx) {
        int sendid = req.jinfo[jidx].sendid;
        reqdata->setData(strokes[jidx], sendid * 2);
    }

    reqdata->setData(duration, 60);
    reqdata->addChecksum();

    buff.size = reqdata->getTotalLen();

    return true;
}

template<>
bool makeBuffImpl<aero4::SendBuff>(BuffRaw &buff, int msid, const motion_player::ArmInfo &req, const int16_t strokes[], uint16_t duration) {
    aero4::SendBuff *reqdata = reinterpret_cast<aero4::SendBuff*>(buff.data);
    reqdata->init();
    reqdata->header.data[0] = 0xFD;
    reqdata->header.data[1] = 0xDF;
    reqdata->cmd = 0x14;
    reqdata->msid = msid;

    for (size_t jidx = 0; jidx < req.jinfo.size(); ++jidx) {
        int sendid = req.jinfo[jidx].sendid;
        reqdata->setData(strokes[jidx], sendid * 2);
    }

    reqdata->setData(duration, 56);
    reqdata->addChecksum();

    buff.size = reqdata->getTotalLen();

    return true;
}


void makeBuff(const std::string &protocol, BuffRaw &buff, int msid, const motion_player::ArmInfo &req, const int16_t strokes[], uint16_t duration) {
    if (protocol == "aero3") {
        makeBuffImpl<aero3::SendBuff>(buff, msid, req, strokes, duration);
    } else {
        makeBuffImpl<aero4::SendBuff>(buff, msid, req, strokes, duration);
    }
}

}
