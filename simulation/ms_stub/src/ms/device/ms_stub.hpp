#pragma once

#include "tools.hpp"
#include "ms_data.hpp"
#include "serial.hpp"
#include "data_stock.hpp"

class MSStubSingle {
public:
    MSStubSingle(int msid) :
            msid(msid) {
    }

    virtual ~MSStubSingle() {
    }

    void setJointEncRange(int jidx, int16_t min, int16_t max) {
        std::lock_guard < std::mutex > (data.mtx);
        data.joint[jidx].pos_min = min;
        data.joint[jidx].pos_max = max;
    }

    void setStatusData(int bit, bool value) {
        std::lock_guard < std::mutex > (data.mtx);
        size_t ofst = std::floor(bit / 8.0);
        if (ofst >= sizeof(data.stat_detail.data)) {
            return;
        }

        int pos = bit % 8;
        auto mask = createBitmask(pos, 1);

        if (value == true) {
            data.stat_detail.data[ofst] |= mask;
        } else {
            data.stat_detail.data[ofst] &= ~mask;
        }

        dump("status info", sizeof(data.stat_detail.data), data.stat_detail.data);
    }

    void update(double cycle_sec) {
        std::lock_guard < std::mutex > (data.mtx);
        for (int idx = 0; idx < AXIS_MAX; ++idx) {

            //速度が入っている場合は、とりあえず動く
            double pos_next = data.joint[idx].pos_cur + data.joint[idx].vel_cur * cycle_sec;

            //時間指定による目標位置までの移動の場合
            if (!std::isnan(data.joint[idx].pos_tgt)) {
                if ((data.joint[idx].pos_cur < data.joint[idx].pos_tgt && data.joint[idx].pos_tgt <= pos_next) || (pos_next <= data.joint[idx].pos_tgt && data.joint[idx].pos_tgt < data.joint[idx].pos_cur)) {
                    data.joint[idx].vel_cur = 0;
                    data.joint[idx].pos_tgt = std::numeric_limits<double>::quiet_NaN();
                }
            }

            //エンコーダの限界値でオーバーフロー
            if (pos_next > data.joint[idx].pos_max) {
                pos_next += (data.joint[idx].pos_min - data.joint[idx].pos_max);
            } else if (pos_next < data.joint[idx].pos_min) {
                pos_next += (data.joint[idx].pos_max - data.joint[idx].pos_min);
            }

            data.joint[idx].pos_cur = pos_next;
            data.joint[idx].pos = data.joint[idx].pos_cur;
        }
    }

    int getMsId() const{
        return msid;
    }

    bool accept_interf(Serial &serial, uint8_t *recvd_raw, size_t len) {
        stock.append(recvd_raw, len);
        bool ret = true;
        std::lock_guard < std::mutex > (data.mtx);

        while (1) {
            auto expect_size = getExpectSize(stock.front());
            uint8_t recvd_tmp[256];
            if (!stock.read(recvd_tmp, expect_size)) {
                break;
            }

            if (!accept(serial, recvd_tmp, expect_size)) {
                ret = false; //一つでも破棄したら、次に回す
            }
        }

        return ret;
    }

    virtual size_t getExpectSize(const uint8_t *buff) const {
        return 0;
    }

protected:
    MSData& getMsData() {
        return data;
    }

    virtual bool accept(Serial &serial, uint8_t *recvd_raw, size_t len) {
        return false;
    }

private:
    uint8_t createBitmask(size_t from, size_t size) {
        uint8_t st_mask = ~((0x01 << from) - 1);
        uint8_t end_mask = ((0x01 << (from + size)) - 1);
        auto mask = (st_mask & end_mask);
        mask = ((mask & 0x55) << 1) | ((mask & 0xAA) >> 1);
        mask = ((mask & 0x33) << 2) | ((mask & 0xCC) >> 2);
        mask = (mask << 4) | (mask >> 4);
        return mask;
    }

private:
    int msid = 0;
    MSData data;
    DataStock stock;
};
