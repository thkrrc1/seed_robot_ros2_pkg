#pragma once

#include <memory>

struct DataStock {
    static constexpr int mlen = 256;
    int size = 0; // 格納サイズ
    uint8_t data[mlen] = { 0 };

    int maxlen() const {
        return mlen;
    }

    int len() const {
        return size;
    }

    uint8_t* front() {
        return data;
    }

    bool read(uint8_t *buff, int len) {
        if (size < len) {
            return false;
        }
        memcpy(buff, data, len);
        memcpy(data, data + len, mlen - len); //前に詰める
        size -= len;
        return true;
    }

    bool append(uint8_t *buff, int len) {
        if (size + len > mlen) {
            return false;
        }
        memcpy(&data[size], buff, len);
        size += len;
        return true;
    }
};
