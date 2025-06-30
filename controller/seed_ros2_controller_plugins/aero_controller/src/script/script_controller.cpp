#include <chrono>
#include <rt_logger/logger.hpp>
#include "script_controller.hpp"
#include "aero_controller.hpp"
#include "get_dio.hpp"

namespace aero_controller {


bool ScriptController::waitForDone(AeroController *controller, ScriptContext &context) {
    double time_sec = 0;
    auto time_st = std::chrono::system_clock::now();

    while (1) {
        auto time_end = std::chrono::system_clock::now();
        time_sec = std::chrono::duration<double>(time_end - time_st).count();
        if (time_sec >= context.timeout_sec) {
            for (auto& [msid, ms_context] : context.ms_contexts) {
                for (auto& [mcid, mc_context] : ms_context.mc_contexts) {
                    if (mc_context.cur_dio == mc_context.run_dio) {
                        //実行中の軸は、タイムアウト
                        mc_context.cur_dio = SCRIPT_TIMEOUT;
                    }
                }
            }
            break;
        }

        //DIO取得
        controller->sendDioCommand();
        bool wait_timeout = false;
        auto lock = dio_recvd.waitForData(0.1, wait_timeout);
        if(wait_timeout){
            continue;
        }

        for (auto& [msid, ms_context] : context.ms_contexts) {
            for (auto& [mcid, mc_context] : ms_context.mc_contexts) {
                if (mc_context.cur_dio != SCRIPT_CANCELED) {
                    mc_context.cur_dio = dio_recvd.get(msid, mcid);
                }
            }
        }

        lock.unlock();

        //終了判定
        if (context.isCanceled()) {
            LOG_INFO_STREAM()<<"canceled"<<LOG_END;
            break;
        } else if (context.isCompleted()) {
            LOG_INFO_STREAM()<<"completed"<<LOG_END;
            break;
        }
    }


    return true;
}

bool ScriptController::runScript(AeroController *controller, ScriptContext &context) {
    mtx_ensure_run.lock();

    //DIOのデータを無理やり書き換えて、キャンセルされたことにする
    std::map<int, DioRecvDataMs> dio_data;
    for (auto& [msid, ms_context] : context.ms_contexts) {
        for (auto& [mcid, mc_context] : ms_context.mc_contexts) {
            dio_data[msid].dio_mc[mcid].value = SCRIPT_CANCELED;
        }
    }

    //実行済みの他のスクリプト実行を終了させる
    dio_recvd.lockReset(dio_data);
    usleep(100);
    dio_recvd.unlock(dio_data);

    //スクリプトコマンドを送信する
    controller->sendScriptCommand(context);

    usleep(50000); // スクリプト実行中になるまで待つ

    mtx_ensure_run.unlock();

    //スクリプトコマンドの終了を待つ
    waitForDone(controller, context);

    return true;
}

}
