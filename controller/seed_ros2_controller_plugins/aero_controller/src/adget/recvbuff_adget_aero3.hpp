#pragma once

#include <sstream>
#include <vector>
#include <seed_ros2_controller/aero/version3/recv_buff_aero3.hpp>

namespace aero_controller {

struct RecvBuffAdGetAero3: public aero3::RecvBuff {
    bool is_adget_resp() const{
        return cmd == 0x44;
    }

    // 2byteのValue1~30のデータに変換する
    bool get_ad_value(std::vector<int>& value_data) const {
        value_data.clear();
        // 末尾にあるNone:2byte, checsum:1byteの不要なデータを取り除く
        const int unused_data = 3;
        const int datasize = (sizeof(data) /sizeof(uint8_t)) - unused_data;
        
        int vi=0;
        for(int di = 0; di < datasize; di += 2){
            value_data.push_back((static_cast<uint16_t>(data[di]) << 8) | static_cast<uint16_t>(data[di+1]));
            vi++;
        }
        return true;
    }
};

}
