#pragma once

#include "seed_ros2_controller/aero/type_traits.hpp"
#include "seed_ros2_controller/aero/header.hpp"

namespace aero4{
struct RecvBuff{
    Header header;
    uint8_t crd = 0;
    uint8_t cmd = 0;
    uint8_t msid = 0;
    uint8_t data[58] = {0};
    uint8_t cs = 0;

    template<class T>
    auto get(int st_idx, T &out) -> std::enable_if<is_byte<T,2>::value, bool>::type  const {
        if(st_idx < 0 || st_idx +1 >= datalen()){
            return false;
        }
        out = *reinterpret_cast<T*>(&data[st_idx]);
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

        int idx_cs = datalen() + 3;
        const uint8_t *data_tmp = &crd;
        for (int idx = 0; idx < idx_cs; idx++){
            cs += data_tmp[idx];
        }

        return (this->cs == static_cast<uint8_t>(~cs));
    }

    int datalen() const{
        return sizeof(data);
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
