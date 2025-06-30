#pragma once

#include <seed_ros2_controller/aero/version3/recv_buff_aero3.hpp>

namespace aero_controller {

struct RecvBuffDioAero3: public aero3::RecvBuff {
    bool is_dio_resp() const{
        return cmd == 0x45;
    }

    int get(int idx) const {
        if (idx < 0 || idx * 2 + 1 >= datalen()) {
            return -1;
        }
        auto value = static_cast<uint16_t>((data[idx * 2] << 8) + data[idx * 2 + 1]);
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
