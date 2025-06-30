#include <string>
#include "get_version.hpp"

namespace aero_controller {

template<class T>
bool makeBuffGetVersionImpl(BuffRaw &buff, uint8_t msid, uint8_t mcid) {
    return false;
}

template<>
bool makeBuffGetVersionImpl<aero3::SendBuff>(BuffRaw &buff, uint8_t msid, uint8_t mcid) {
    buff.clear();
    aero3::SendBuff *reqdata = reinterpret_cast<aero3::SendBuff*>(buff.data);
    reqdata->init();
    reqdata->header.data[0] = 0xFD;
    reqdata->header.data[1] = 0xDF;
    reqdata->cmd = 0x51;
    reqdata->mcid = mcid;
    reqdata->addChecksum();

    buff.size = reqdata->getTotalLen();

    return true;
}

template<>
bool makeBuffGetVersionImpl<aero4::SendBuff>(BuffRaw &buff, uint8_t msid, uint8_t mcid) {
    buff.clear();
    aero4::SendBuff *reqdata = reinterpret_cast<aero4::SendBuff*>(buff.data);
    reqdata->init();
    reqdata->header.data[0] = 0xFD;
    reqdata->header.data[1] = 0xDF;
    reqdata->cmd = 0x51;
    reqdata->msid = msid;
    reqdata->setData(mcid,0);
    reqdata->addChecksum();
    buff.size = reqdata->getTotalLen();

    return true;
}

bool makeBuffGetVersion(const std::string &protocol, BuffRaw &buff, uint8_t msid, uint8_t mcid) {
    if (protocol == "aero3") {
        return makeBuffGetVersionImpl<aero3::SendBuff>(buff, msid, mcid);
    } else {
        return makeBuffGetVersionImpl<aero4::SendBuff>(buff, msid, mcid);
    }
}
}
