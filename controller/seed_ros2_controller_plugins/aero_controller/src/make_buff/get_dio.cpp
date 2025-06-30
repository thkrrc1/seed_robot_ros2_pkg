#include <string>
#include "get_dio.hpp"

namespace aero_controller {

template<class T>
bool makeBuffGetDioImpl(BuffRaw &buff, int msid) {
    return false;
}

template<>
bool makeBuffGetDioImpl<aero3::SendBuff>(BuffRaw &buff, int msid) {
    buff.clear();
    aero3::SendBuff *reqdata = reinterpret_cast<aero3::SendBuff*>(buff.data);
    reqdata->init();
    reqdata->header.data[0] = 0xFD;
    reqdata->header.data[1] = 0xDF;
    reqdata->cmd = 0x45;
    reqdata->mcid = 0x00;
    reqdata->addChecksum();

    buff.size = reqdata->getTotalLen();

    return true;
}

template<>
bool makeBuffGetDioImpl<aero4::SendBuff>(BuffRaw &buff, int msid) {
    buff.clear();
    aero4::SendBuff *reqdata = reinterpret_cast<aero4::SendBuff*>(buff.data);
    reqdata->init();
    reqdata->header.data[0] = 0xFD;
    reqdata->header.data[1] = 0xDF;
    reqdata->cmd = 0x45;
    reqdata->msid = msid;

    reqdata->addChecksum();
    buff.size = reqdata->getTotalLen();

    return true;
}

bool makeBuffGetDio(const std::string &protocol, BuffRaw &buff, int msid) {
    if (protocol == "aero3") {
        return makeBuffGetDioImpl<aero3::SendBuff>(buff, msid);
    } else {
        return makeBuffGetDioImpl<aero4::SendBuff>(buff, msid);
    }
}
}
