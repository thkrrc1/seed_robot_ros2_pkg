#include "robotstatus.hpp"
#include <seed_ros2_controller/aero/version3/send_buff_aero3.hpp>
#include <seed_ros2_controller/aero/version3/recv_buff_aero3.hpp>
#include <seed_ros2_controller/aero/version4/send_buff_aero4.hpp>
#include <seed_ros2_controller/aero/version4/recv_buff_aero4.hpp>

namespace aero_controller {
template<class T>
bool makeBuffRobotStatusImpl(BuffRaw &buff, int msid) {
    return false;
}

template<>
bool makeBuffRobotStatusImpl<aero3::SendBuff>(BuffRaw &buff, int msid) {
    aero3::SendBuff *reqdata = reinterpret_cast<aero3::SendBuff*>(buff.data);
    reqdata->init();
    reqdata->header.data[0] = 0xFD;
    reqdata->header.data[1] = 0xDF;
    reqdata->cmd = 0x52;
    reqdata->mcid = 0x00;
    reqdata->addChecksum();

    buff.size = reqdata->getTotalLen();

    return true;
}

template<>
bool makeBuffRobotStatusImpl<aero4::SendBuff>(BuffRaw &buff, int msid) {
    aero4::SendBuff *reqdata = reinterpret_cast<aero4::SendBuff*>(buff.data);
    reqdata->init();
    reqdata->header.data[0] = 0xFD;
    reqdata->header.data[1] = 0xDF;
    reqdata->cmd = 0x52;
    reqdata->msid = msid;
    reqdata->addChecksum();

    buff.size = reqdata->getTotalLen();

    return true;
}

bool makeBuffRobotStatus(const std::string &protocol, BuffRaw &buff, int msid) {
    if (protocol == "aero3") {
        return makeBuffRobotStatusImpl<aero3::SendBuff>(buff, msid);
    } else {
        return makeBuffRobotStatusImpl<aero4::SendBuff>(buff, msid);
    }
}


template<class T>
bool getDataImpl(const BuffRaw &buff, aero_controller_msgs::msg::RobotStatus &status) {
    return false;
}



template<>
bool getDataImpl<aero3::RecvBuff>(const BuffRaw &buff, aero_controller_msgs::msg::RobotStatus &status) {
    auto *respdata = reinterpret_cast<const aero3::RecvBuff*>(buff.data);
    if (respdata->cmd != 0x52) {
        return false;
    } else if (respdata->len != 64) {
        return false;
    }

    for (size_t idx = 0; idx < status.data.size(); ++idx) {
        status.data[idx] = respdata->data[idx];
    }
    return true;
}

template<>
bool getDataImpl<aero4::RecvBuff>(const BuffRaw &buff, aero_controller_msgs::msg::RobotStatus &status) {
    auto *respdata = reinterpret_cast<const aero4::RecvBuff*>(buff.data);
    if (respdata->cmd != 0x52) {
        return false;
    }

    // status.msid = respdata->msid;
    memcpy(&status.data[0], respdata->data, status.data.size());
    return true;
}

bool getDataRobotStatus(const std::string &protocol, const BuffRaw &buff, aero_controller_msgs::msg::RobotStatus &status) {
    if (protocol == "aero3") {
        return getDataImpl<aero3::RecvBuff>(buff, status);
    } else {
        return getDataImpl<aero4::RecvBuff>(buff, status);
    }
}

}
