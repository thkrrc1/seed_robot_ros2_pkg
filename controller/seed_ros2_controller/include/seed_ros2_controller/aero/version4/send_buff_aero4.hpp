#pragma once

#include <seed_ros2_controller/aero/type_traits.hpp>
#include <seed_ros2_controller/aero/header.hpp>

namespace aero4 {
struct SendBuff {
    Header header;
    uint8_t crd = 0;
    uint8_t cmd = 0;
    uint8_t msid = 0;
    uint8_t data[58] = { 0 };
    uint8_t cs = 0;

    uint8_t* serialize() {
        return reinterpret_cast<uint8_t*>(this);
    }

    SendBuff() {
        init();
    }

    void init() {
        memset(data, 0x00, sizeof(data));
    }

    template<class T>
    auto setData(T *data, int data_len, int st_idx) -> std::enable_if<is_byte<T,2>::value,int>::type {
        uint8_t *data_le = reinterpret_cast<uint8_t*>(data); // little endian
        for (int idx = 0; idx < data_len; ++idx) {
            this->data[idx * 2 + st_idx] = data_le[idx * 2];
            this->data[idx * 2 + 1 + st_idx] = data_le[idx * 2 + 1];
        }
        int next_idx = data_len * 2;

        return next_idx;
    }

    template<class T>
    auto setData(T data, int st_idx) -> std::enable_if<is_byte<T,2>::value,int>::type {
        uint8_t *data_le = reinterpret_cast<uint8_t*>(&data);
        this->data[st_idx] = data_le[0];
        this->data[st_idx + 1] = data_le[1];
        int next_idx = st_idx + 2;
        return next_idx;
    }

    template<class T>
    auto setData(T data, int n, int st_idx) -> std::enable_if<is_byte<T,2>::value,int>::type {
        uint8_t *data_le = reinterpret_cast<uint8_t*>(&data);
        for (int idx = 0; idx < n; ++idx) {
            this->data[st_idx + idx * 2] = data_le[0];
            this->data[st_idx + 1 + idx * 2] = data_le[1];
        }
        int next_idx = st_idx + n * 2;
        return next_idx;
    }

    template<class T>
    auto setData(T *data, int data_len, int st_idx) -> std::enable_if<is_byte<T,1>::value,int>::type {
        uint8_t *u8d = reinterpret_cast<uint8_t*>(data);
        for (int idx = 0; idx < data_len; ++idx) {
            this->data[idx + st_idx] = u8d[idx];
        }
        int next_idx = data_len + st_idx;
        return next_idx;
    }

    template<class T>
    auto setData(T data, int st_idx) -> std::enable_if<is_byte<T,1>::value,int>::type {
        uint8_t *u8d = reinterpret_cast<uint8_t*>(&data);
        this->data[st_idx] = *u8d;
        int next_idx = st_idx + 1;
        return next_idx;
    }

    int datalen() const{
        return sizeof(data);
    }

    int getTotalLen() const {
        return 64;
    }

    void addChecksum() {
        unsigned int cs = 0;

        int idx_cs = getTotalLen() - sizeof(Header) - 1;
        uint8_t *data_tmp = &crd;
        for (int idx = 0; idx < idx_cs; idx++) {
            cs += data_tmp[idx];
        }
        this->cs = static_cast<uint8_t>(~cs);
    }
};
}
