#pragma once

#include <seed_ros2_controller/command_interface/other_command/other_cmd_buff.hpp>


#include <aero_controller_msgs/srv/servo.hpp>
#include <seed_ros2_controller/aero/version3/send_buff_aero3.hpp>
#include <seed_ros2_controller/aero/version4/send_buff_aero4.hpp>

namespace aero_controller {

template<class T>
void makeBuffImpl(BuffRaw &buff, int msid, aero_controller_msgs::srv::Servo::Request &req) {
}

template<>
void makeBuffImpl<aero3::SendBuff>(BuffRaw &buff, int msid, aero_controller_msgs::srv::Servo::Request &req) {
    buff.clear();
    aero3::SendBuff *reqdata = reinterpret_cast<aero3::SendBuff*>(buff.data);
    reqdata->init();
    reqdata->header.data[0] = 0xFD;
    reqdata->header.data[1] = 0xDF;
    reqdata->cmd = 0x21;
    reqdata->mcid = 0x00;

    int next_idx = 0;
    uint16_t on = 0x0000;
    uint16_t nop = 0x7FFF;
    if (req.type == aero_controller_msgs::srv::Servo::Request::SERVO_ON) {
        on = 0x0001;
    }

    next_idx = reqdata->setData(on, 30, next_idx);
    next_idx = reqdata->setData(nop, next_idx);
    reqdata->addChecksum();

    buff.size = reqdata->getTotalLen();
}

template<>
void makeBuffImpl<aero4::SendBuff>(BuffRaw &buff, int msid, aero_controller_msgs::srv::Servo::Request &req) {
    buff.clear();
    aero4::SendBuff *reqdata = reinterpret_cast<aero4::SendBuff*>(buff.data);
    reqdata->init();
    reqdata->header.data[0] = 0xFD;
    reqdata->header.data[1] = 0xDF;
    reqdata->cmd = 0x21;
    reqdata->msid = msid;

    int next_idx = 0;
    uint16_t on = 0x0000;
    uint16_t nop = 0xFF00;
    if (req.type == aero_controller_msgs::srv::Servo::Request::SERVO_ON) {
        on = 0x0001;
    }

    next_idx = reqdata->setData(on, 28, next_idx);
    next_idx = reqdata->setData(nop, next_idx);
    reqdata->addChecksum();

    buff.size = reqdata->getTotalLen();
}

void makeBuff(const std::string &protocol, BuffRaw &buff, int msid, aero_controller_msgs::srv::Servo::Request &req) {
    if (protocol == "aero3") {
        makeBuffImpl<aero3::SendBuff>(buff, msid, req);
    } else {
        makeBuffImpl<aero4::SendBuff>(buff, msid, req);
    }
}
}
