#include <string>
#include "get_ad.hpp"

namespace aero_controller {

template<class T>
bool makeBuffGetAdImpl(BuffRaw &buff, int msid, int ad_no) {
    return false;
}

template<>
bool makeBuffGetAdImpl<aero3::SendBuff>(BuffRaw &buff, int msid, int ad_no) {
    buff.clear();
    aero3::SendBuff *reqdata = reinterpret_cast<aero3::SendBuff*>(buff.data);
    reqdata->init();
    reqdata->header.data[0] = 0xFD;
    reqdata->header.data[1] = 0xDF;
    reqdata->cmd = 0x44;
    reqdata->mcid = 0x00;
    uint8_t ad_no_8 =static_cast<uint8_t>(ad_no);
    reqdata->setData(ad_no_8, 0);
    reqdata->addChecksum();
    
    buff.size = reqdata->getTotalLen();

    return true;
}

template<>
bool makeBuffGetAdImpl<aero4::SendBuff>(BuffRaw &buff, int msid, int ad_no) {
    buff.clear();
    aero4::SendBuff *reqdata = reinterpret_cast<aero4::SendBuff*>(buff.data);
    reqdata->init();
    reqdata->header.data[0] = 0xFD;
    reqdata->header.data[1] = 0xDF;
    reqdata->cmd = 0x44;
    reqdata->msid = msid;
    uint8_t ad_no_8 =static_cast<uint8_t>(ad_no);
    reqdata->setData(ad_no_8, 0);
    reqdata->addChecksum();

    buff.size = reqdata->getTotalLen();

    return true;
}

bool makeBuffGetAd(const std::string &protocol, BuffRaw &buff, int msid, int ad_no) {
    if (protocol == "aero3") {
        return makeBuffGetAdImpl<aero3::SendBuff>(buff, msid, ad_no);
    } else {
        return makeBuffGetAdImpl<aero4::SendBuff>(buff, msid, ad_no);
    }
}
}
