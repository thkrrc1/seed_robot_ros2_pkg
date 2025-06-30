#pragma once

#include <memory>

#include "log_sink_thread.hpp"
#include "logger_core.hpp"
#include "singleton.hpp"
#include "macro.hpp"

namespace rtlogger {
inline void setLoggerCore(std::shared_ptr<LoggerCore> logger) {
    if(Singleton<LogSinkThread>::finished())return;
    Singleton<LogSinkThread>::getInstance().setLoggerCore(logger);
}

inline void insertLog(const LogFormat &fmt) {
    if(Singleton<LogSinkThread>::finished())return;
    Singleton<LogSinkThread>::getInstance().insertLog(fmt);
}

inline void setProcessName(const std::string &name) {
    if(Singleton<LogSinkThread>::finished())return;
    Singleton<LogSinkThread>::getInstance().setProcessName(name);
}
}

#define LOGGER_INIT\
        SingletonHolder<rtlogger::LogSinkThread> holder;
