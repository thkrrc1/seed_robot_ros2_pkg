#pragma once

#include <cstdint>
#include <cstring>

struct BuffRaw{
    uint8_t data[256] = {0};
    int size = 0;

    void clear(){
        size = 0;
        memset(data,0x00,sizeof(data));
    }

    void addChecksum(){
        unsigned int cs = 0;
        for (int idx = 2; idx < size - 1; idx++){
            cs += data[idx];
        }
        data[size - 1] = static_cast<uint8_t>(~cs);
    }

    bool isvalid() const {
        unsigned int cs = 0;
        for (int idx = 2; idx < size - 1; idx++) {
            cs += data[idx];
        }
        return (data[size - 1] == static_cast<uint8_t>(~cs));
    }

};

struct BuffList{
    static constexpr int capacity = 10;
    BuffRaw buf[capacity];
    int start_idx = 0;
    int size = 0;

    int cmd_capacity(){
        return capacity;
    }
};
