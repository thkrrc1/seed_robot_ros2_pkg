#pragma once

#include "seed_ros2_controller/aero/type_traits.hpp"
#include "seed_ros2_controller/aero/header.hpp"

namespace aero3{
struct SendBuff{
    Header header;
    uint8_t len = 0;
    uint8_t cmd = 0;
    uint8_t mcid = 0;
    uint8_t data[63] = {0};

    uint8_t* serialize(){
        return reinterpret_cast<uint8_t*>(this);
    }

    SendBuff(){
        init();
    }

    void init() {
        for (unsigned int idx = 0; idx < sizeof(data); ++idx) {
            if (idx % 2 == 0) {
                data[idx] = 0x7F;
            } else {
                data[idx] = 0xFF;
            }
        }
        len = 2; //cmd + mcidの2バイトは最低必要
    }

    template<class T>
    auto setData(T* data,int data_len,int st_idx) -> std::enable_if<is_byte<T,2>::value,int>::type{
        uint16_t* u16d = reinterpret_cast<uint16_t*>(data);
        for (int idx = 0; idx < data_len; ++idx) {
            this->data[idx * 2 + st_idx] = u16d[idx] >> 8;
            this->data[idx * 2 + 1 + st_idx] = u16d[idx];
        }
        int next_idx = data_len * 2;

        len = next_idx + 2; // data_size + cmd + mcid
        return next_idx;
    }

    template<class T>
    auto setData(T data,int st_idx) -> std::enable_if<is_byte<T,2>::value,int>::type{
        uint16_t* u16d = reinterpret_cast<uint16_t*>(&data);
        this->data[st_idx] = *u16d >> 8;
        this->data[st_idx + 1] = *u16d;
        int next_idx = st_idx+2;
        len = next_idx + 2; // data_size + cmd + mcid
        return next_idx;
    }


    template<class T>
    auto setData(T data, int n, int st_idx) -> std::enable_if<is_byte<T,2>::value,int>::type {
        uint16_t *u16d = reinterpret_cast<uint16_t*>(&data);
        for (int idx = 0; idx < n; ++idx) {
            this->data[st_idx + idx * 2] = *u16d >> 8;
            this->data[st_idx + 1 + idx * 2] = *u16d;
        }
        int next_idx = st_idx + n * 2;
        len = next_idx + 2; // data_size + cmd + mcid
        return next_idx;
    }


    template<class T>
    auto setData(T* data,int data_len,int st_idx) -> std::enable_if<is_byte<T,1>::value,int>::type{
        uint8_t* u8d = reinterpret_cast<uint8_t*>(data);
        for (int idx = 0; idx < data_len; ++idx) {
            this->data[idx + st_idx] = u8d[idx];
        }
        int next_idx = data_len + st_idx;

        len = next_idx + 2;// data_size + cmd + mcid
        return next_idx;
    }


    template<class T>
    auto setData(T data, int st_idx) -> std::enable_if<is_byte<T,1>::value,int>::type {
        uint8_t *u8d = reinterpret_cast<uint8_t*>(&data);
        this->data[st_idx] = *u8d;
        int next_idx = st_idx + 1;
        len = next_idx + 2; // data_size + cmd + mcid
        return next_idx;
    }

    int getTotalLen() const{
        return len + 4; // header*2 + len + checksum
    }

    /**
     * ボディー部のデータが存在するか
     */
    bool existBody(){
        return len > 2; // mcid + checksum以外のデータが入っているかどうか
    }

    /**
     * データをいっぱいにする
     */
    void cram(){
        len = 64;
    }

    void addChecksum(){
        unsigned int cs = 0;

        cs += len;
        uint8_t* d = &cmd;
        for (int idx = 0; idx < len; idx++){
            cs += d[idx];
        }

        d[len] = static_cast<uint8_t>(~cs);
    }
};
}
