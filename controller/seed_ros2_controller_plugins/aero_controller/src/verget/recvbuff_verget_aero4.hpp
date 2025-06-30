#pragma once

#include <sstream>
#include <iomanip>
#include <seed_ros2_controller/aero/version4/recv_buff_aero4.hpp>

namespace aero_controller {

struct RecvBuffVerGetAero4: public aero4::RecvBuff {
    bool is_verget_resp() const{
        return cmd == 0x51;
    }

    //非RT用
    std::string get() const {
        std::string version;
        for(int idx = 0;idx < 5;++idx){
            std::ostringstream ss;
            ss << std::hex << std::setw(2) << std::setfill('0') << int(data[idx]);
            version += ss.str();
        }
        return version;
    }

    int size() const {
        return datalen() / 2;
    }

};

}
