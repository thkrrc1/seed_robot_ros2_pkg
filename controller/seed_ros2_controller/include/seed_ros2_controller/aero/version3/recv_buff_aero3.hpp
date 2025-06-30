#pragma once

#include <seed_ros2_controller/aero/type_traits.hpp>
#include <seed_ros2_controller/aero/header.hpp>

namespace aero3{
struct RecvBuff{
    Header header;
    uint8_t len = 0; //!< cmd〜チェックサムの一つ手前までのデータ長
    uint8_t cmd = 0;
    uint8_t mcid = 0;
    uint8_t data[63] = {0};

    template<class T>
    auto get(int st_idx, T &out) -> std::enable_if<is_byte<T,2>::value, bool>::type  const {
        if(st_idx < 0 || st_idx +1 >= datalen()){
            return false;
        }
        out = static_cast<T>((data[st_idx] << 8) + data[st_idx + 1]);
        return true;
    }

    template<class T>
    auto get(int st_idx, T &out) -> std::enable_if<is_byte<T,1>::value, bool>::type  const {
        if(st_idx < 0 || st_idx >= datalen()){
            return false;
        }
        out = data[st_idx];
        return true;
    }

    bool isvalid() const{
        unsigned int cs = 0;

        cs += len;
        cs += cmd;
        cs += mcid;
        for (int idx = 0; idx < datalen(); idx++){
            cs += data[idx];
        }

        return (data[datalen()] == static_cast<uint8_t>(~cs));
    }

    int datalen() const{
        return len - 2; //checksumを除いたデータ部の長さ
    }
};


struct MCState{
    int16_t pos = 0;
    int16_t current = 0;
    int8_t temp = 0;
    int8_t voltage = 0;
};

struct MSState{
    static constexpr int mc_num_max = 100;//!< 一つのmsにつながるMCの数の最大値(64以上あればOK)
    MCState mcval[mc_num_max] = {0};
    uint16_t status = 0;
};
}
