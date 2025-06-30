#pragma once

#include <thread>
#include <atomic>
#include <memory>
#include <condition_variable>
#include <unistd.h>
#include <cstring>
#include "log_format.hpp"
#include "logger_core.hpp"
#include "swap_buffer.hpp"
#include "singleton.hpp"
namespace rtlogger {

class LogSinkThread {
public:
    LogSinkThread() {
        shutdown.store(false);
        outputThread = std::thread(&LogSinkThread::output, this);
        usleep(100);//出力スレッドが立ち上がるまで待つ
    }

    ~LogSinkThread() {
        shutdown.store(true);
        cond.notify_one();
        if (outputThread.joinable()) {
            outputThread.join();
        }
    }

    void setLoggerCore(std::shared_ptr<LoggerCore> logger) {
        std::lock_guard<std::mutex> lk(mtx);
        this->logger = logger;
    }

    /**
     * ログの発行とかぶらないように呼び出すこと。
     */
    void setProcessName(const std::string& name){
        process_name = name;
    }

    /**
     * すでに作成したログを挿入する場合
     */
    void insertLog(const LogFormat &fmt) {
        buff.writeFromA(fmt);
        cond.notify_one();
    }

    /**
     * LOG_マクロから挿入する場合
     */
    void insertLogFromStream(LogFormat &fmt) {
        strncpy(fmt.pname, process_name.c_str(), sizeof(fmt.pname));
        insertLog(fmt);
    }

private:
    void output_log(){
        auto fmt = buff.readFromB();
        if (!logger) {
            return;
        }
        while (fmt) {
            logger->putLog(*fmt);
            fmt = buff.readFromB();
        }
    }

    void output() {
        while (!shutdown.load()) {
            std::unique_lock < std::mutex > lk(mtx);
            cond.wait(lk);
            output_log();
        }

        //残りのログを吐き出す
        output_log();
    }

private:
    std::shared_ptr<LoggerCore> logger = nullptr;

    rtlogger::SwapBuffer<LogFormat, 100, 10> buff;
    std::thread outputThread;
    std::atomic<bool> shutdown;
    std::string process_name;

    std::mutex mtx;
    std::condition_variable cond;
};
}
