#pragma once

#include <thread>
#include <atomic>
#include <cstring>

#include "tools.hpp"
#include "serial.hpp"
#include "timer_lib.hpp"

#include "ms_stub_aero3.hpp"
#include "ms_stub_aero4.hpp"

class MSChainUSB {
public:
    MSChainUSB(std::string &dev_path) {
        serial.open(dev_path);
        updata_thread = std::thread(&MSChainUSB::update, this);
        cmd_thread = std::thread(&MSChainUSB::exec, this);
    }

    ~MSChainUSB() {
        serial.close();
        shutdown.store(true);
        if (cmd_thread.joinable()) {
            cmd_thread.join();
        }

        if (updata_thread.joinable()) {
            updata_thread.join();
        }

        auto itr = mslist.begin();
        while(itr != mslist.end()){
            auto msstub = itr->second;
            itr = mslist.erase(itr);
            delete msstub;
        }
    }

    void setJointEncRange(int msid, int jidx, int16_t min, int16_t max) {
        if (mslist.count(msid) != 0) {
            mslist[msid]->setJointEncRange(jidx, min, max);
        }
    }

    void setStatusData(int msid, int bit, bool value) {
        if (mslist.count(msid) != 0) {
            mslist[msid]->setStatusData(bit, value);
        }
    }

    std::vector<int> getMsIds() {
        std::vector<int> ret;
        for (auto& [msid, msdata] : mslist) {
            ret.push_back(msid);
        }
        return ret;
    }

    void addMs(int id, const std::string &protocol) {
        if (mslist.count(id) == 0) {
            MSStubSingle* ms = nullptr;
            if (protocol == "aero4") {
                ms = new aero4::MSStubSingleAero4(id);
            } else {
                ms = new aero3::MSStubSingleAero3(id);
            }
            mslist.emplace(id, ms);
        }
    }

    template<class T>
    void sendOtherCmd(T *cmd) {
        serial.write(cmd, cmd->getTotalLen());
    }

private:
    void exec() {
        timespec t1_prev;
        auto tm = getTime();
        t1_prev = tm;

        uint8_t recvd_raw[256] = { 0 };

        while (!shutdown.load()) {
            auto rlen = serial.read(recvd_raw, sizeof(recvd_raw));
            if (rlen <= 0) {
                continue;
            }

            for (auto& [msid, msstub] : mslist) {
                if (msstub->accept_interf(serial, recvd_raw, rlen)) {
                    break;
                }
            }
        }
    }

    void update() {
        double period_sec = 0.005; //[s]
        long period_ns = period_sec * NSEC_PER_SEC;

        auto tm = getTime();
        while (!shutdown.load()) {
            getNextTime(tm, period_ns);
            sleepUntil(tm);

            for (auto& [msid, msstub] : mslist) {
                msstub->update(period_sec);
            }
        }
    }

private:
    std::thread cmd_thread;
    std::thread updata_thread;

    std::atomic<bool> shutdown = false;

    std::map<int, MSStubSingle*> mslist;

    Serial serial;
};

