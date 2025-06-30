#pragma once
#include <map>
#include <mutex>
#include <condition_variable>

namespace aero_controller {

enum DIO_VALUE {
    DIO_VALUE_NOP = -1
};

struct DioRecvDataMc {
    int value = DIO_VALUE_NOP;
    bool locked = false;
};

struct DioRecvDataMs {
    std::map<int, DioRecvDataMc> dio_mc;
};

class DioRecvData {

public:
    std::unique_lock<std::mutex> waitForData(double timeout_sec, bool &timeout) {
        std::unique_lock < std::mutex > lk(mtx);
        auto cond_result = cond.wait_for(lk, std::chrono::duration<double>(timeout_sec));

        if (cond_result == std::cv_status::timeout) {
            timeout = true;
        }

        return lk;
    }

    int get(int msid, int mcid) {
        auto ms_itr = dio_data.find(msid);
        if (ms_itr == dio_data.end()) {
            return DIO_VALUE_NOP;
        }

        auto &dio_ms = ms_itr->second;
        auto mc_itr = dio_ms.dio_mc.find(mcid);
        if (mc_itr == dio_ms.dio_mc.end()) {
            return DIO_VALUE_NOP;
        }

        return mc_itr->second.value;
    }

    template<class T>
    bool setRecvData(int msid, const T *data) {
        if (!data->is_dio_resp()) {
            return false;
        }

        std::lock_guard < std::mutex > lk(mtx);
        auto &msdata = dio_data[msid];

        for (int idx = 0; idx < data->size(); ++idx) {
            if (!msdata.dio_mc[idx].locked) {
                msdata.dio_mc[idx].value = data->get(idx);
            }
        }

        cond.notify_all();
        return true;
    }


    void lockReset(const std::map<int, DioRecvDataMs> &dio_data) {
        std::lock_guard < std::mutex > lk(mtx);

        for (auto& [msid, dio_ms] : dio_data) {
            for (auto& [mcid, dio_mc] : dio_ms.dio_mc) {
                this->dio_data[msid].dio_mc[mcid].value = dio_mc.value;
                this->dio_data[msid].dio_mc[mcid].locked = true;
            }
        }
        cond.notify_all();
    }

    void unlock(const std::map<int, DioRecvDataMs> &dio_data) {
        std::lock_guard < std::mutex > lk(mtx);

        for (auto& [msid, dio_ms] : dio_data) {
            for (auto& [mcid, dio_mc] : dio_ms.dio_mc) {
                this->dio_data[msid].dio_mc[mcid].locked = false;
            }
        }
    }

private:
    std::map<int, DioRecvDataMs> dio_data;
    std::mutex mtx;
    std::condition_variable cond;
};

}
