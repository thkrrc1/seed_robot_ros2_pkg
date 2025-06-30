#pragma once

#include <seed_ros2_controller/command_interface/other_command/other_cmd_buff.hpp>

#include "recvbuff_verget_aero3.hpp"
#include "recvbuff_verget_aero4.hpp"
#include "verget_recvd.hpp"

namespace aero_controller {

class AeroController;

class VerGetController {
public:
    std::string getVersion(AeroController *controller, int ms, int mc = 0);

    bool setRecvData(int msid, const std::string &protocol, const BuffRaw *buf_recv, BuffRaw *buf_send) {
        if (protocol == "aero4") {
            return verget_recvd.setRecvData(msid, reinterpret_cast<const RecvBuffVerGetAero4*>(buf_recv->data));
        } else {
            return verget_recvd.setRecvData(msid, reinterpret_cast<const RecvBuffVerGetAero3*>(buf_recv->data));
        }
    }

private:
    std::string cur_version;
    VerGetRecvData verget_recvd;
};

}
