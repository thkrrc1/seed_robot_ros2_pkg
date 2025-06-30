#pragma once

#include <seed_ros2_controller/aero/version4/recv_buff_aero4.hpp>

namespace aero_controller {

struct RecvBuffDioAero4: public aero4::RecvBuff {
    bool is_dio_resp() const{
        return cmd == 0x45;
    }

    int get(int idx) const {
        if (idx < 0 || idx * 2 + 1 >= datalen()) {
            return -1;
        }
        auto value = *reinterpret_cast<const uint16_t*>(&data[idx * 2]);
        if (value == 0x7FFF) {
            return -1;
        }

        return value;
    }

    int size() const {
        return datalen() / 2;
    }

};

}
