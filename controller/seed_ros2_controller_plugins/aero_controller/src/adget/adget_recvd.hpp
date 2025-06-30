#pragma once
#include <map>
#include <mutex>
#include <condition_variable>
#include <vector>

namespace aero_controller {

class AdGetRecvData {

public:
    std::unique_lock<std::mutex> waitForData(double timeout_sec, bool &timeout) {
        std::unique_lock < std::mutex > lk(mtx);
        auto cond_result = cond.wait_for(lk, std::chrono::duration<double>(timeout_sec));

        if (cond_result == std::cv_status::timeout) {
            timeout = true;
        }

        return lk;
    }

   bool get_ad_value(std::vector<int>& data){
        data = value_data;
        return true;
    }

    template<class T>
    bool setRecvData(int msid, const T *data) {
        if (!data->is_adget_resp()) {
            return false;
        }
        data->get_ad_value(value_data);
        std::lock_guard < std::mutex > lk(mtx);
        cond.notify_all();
        return true;
    }

private:
    std::vector<int> value_data;
    std::mutex mtx;
    std::condition_variable cond;
};

}
