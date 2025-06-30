#pragma once

#include <seed_ros2_controller/command_interface/other_command/other_cmd_buff.hpp>

#include "recvbuff_dio_aero3.hpp"
#include "recvbuff_dio_aero4.hpp"
#include "dio_recvd.hpp"

namespace aero_controller {

enum SCRIPT_RESULT{
    SCRIPT_TIMEOUT = -3,
    SCRIPT_CANCELED = -2,
    SCRIPT_NOP = -1,
};

struct ScriptContextMc {
    uint8_t script_no = 0;
    uint8_t arg = 0;
    int run_dio = SCRIPT_NOP;  //!< 実行中を示すDIO番号
    int cur_dio = SCRIPT_NOP;  //!< 現在の応答DIO番号

    bool isCompleted() {
        return cur_dio != run_dio;
    }

    bool isCanceled() {
        return cur_dio == SCRIPT_CANCELED;
    }
};

struct ScriptContextMs {
    std::map<int, ScriptContextMc> mc_contexts;

    bool isCompleted() {
        bool all_completed = true;
        for (auto& [msid, mc_context] : mc_contexts) {
            if (!mc_context.isCompleted()) {
                all_completed = false;
                break;
            }
        }
        return all_completed;
    }

    bool isCanceled() {
        bool all_canceled = true;
        for (auto& [msid, mc_context] : mc_contexts) {
            if (!mc_context.isCanceled()) {
                all_canceled = false;
                break;
            }
        }
        return all_canceled;
    }
};

struct ScriptContext {
    std::map<int, ScriptContextMs> ms_contexts;
    double timeout_sec = 10;

    bool isCompleted() {
        bool all_completed = true;
        for (auto& [msid, ms_context] : ms_contexts) {
            if (!ms_context.isCompleted()) {
                all_completed = false;
                break;
            }
        }
        return all_completed;
    }

    bool isCanceled() {
        bool all_canceled = true;
        for (auto& [msid, ms_context] : ms_contexts) {
            if (!ms_context.isCanceled()) {
                all_canceled = false;
                break;
            }
        }
        return all_canceled;
    }
};

class AeroController;

class ScriptController {
public:
    bool runScript(AeroController *controller, ScriptContext &context);

    bool waitForDone(AeroController *controller, ScriptContext &context);

    bool setRecvData(int msid, const std::string &protocol, const BuffRaw *buf_recv, BuffRaw *buf_send) {
        if (protocol == "aero4") {
            return dio_recvd.setRecvData(msid, reinterpret_cast<const RecvBuffDioAero4*>(buf_recv->data));
        } else {
            return dio_recvd.setRecvData(msid, reinterpret_cast<const RecvBuffDioAero3*>(buf_recv->data));
        }
    }

private:
    DioRecvData dio_recvd;
    std::mutex mtx_ensure_run;
};

}
