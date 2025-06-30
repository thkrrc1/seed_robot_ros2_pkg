#pragma once

#include <seed_ros2_controller/command_interface/other_command/other_cmd_buff.hpp>
#include "recvbuff_adget_aero3.hpp"
#include "adget_recvd.hpp"

namespace aero_controller {

class AeroController;

class AdGetController {
public:
    int getAd(AeroController *controller, int ad_no, std::vector<int>& value_data);

    bool setRecvData(int msid, const std::string &protocol, const BuffRaw *buf_recv, BuffRaw *buf_send) {
        if (protocol == "aero3") {
            return adget_recvd.setRecvData(msid, reinterpret_cast<const RecvBuffAdGetAero3*>(buf_recv->data));
        } else {
            return false;
        }
    }

private:
    AdGetRecvData adget_recvd;
};

}
