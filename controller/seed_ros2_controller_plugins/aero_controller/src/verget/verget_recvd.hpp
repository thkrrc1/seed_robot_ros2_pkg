#pragma once
#include <map>
#include <mutex>
#include <condition_variable>

namespace aero_controller {

class VerGetRecvData {

public:
    std::unique_lock<std::mutex> waitForData(double timeout_sec, bool &timeout) {
        std::unique_lock < std::mutex > lk(mtx);
        auto cond_result = cond.wait_for(lk, std::chrono::duration<double>(timeout_sec));

        if (cond_result == std::cv_status::timeout) {
            timeout = true;
        }

        return lk;
    }

    std::string get_version(){
        return version;
    }

    int get_msid(){
        return msid;
    }

    template<class T>
    bool setRecvData(int msid, const T *data) {
        if (!data->is_verget_resp()) {
            return false;
        }

        std::lock_guard < std::mutex > lk(mtx);
        this->version = data->get();
        this->msid = msid;

        cond.notify_all();
        return true;
    }

private:
    int msid;
    std::string version;
    std::mutex mtx;
    std::condition_variable cond;
};

}
