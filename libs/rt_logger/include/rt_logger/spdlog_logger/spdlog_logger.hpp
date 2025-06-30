#pragma once

#include "../logger_core.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/cfg/env.h"
#include "spdlog/fmt/ostr.h"
#include "sinks/spdlog_sink_base.hpp"

namespace std {
template<bool T1, class T2 = void>
struct disable_if {
};

template<class T2>
struct disable_if<false, T2> {
    typedef T2 type;
};
}

template<class T1,class T2>
struct is_derived_from{
    static const bool value = (!std::is_same<T2,T1>::value) && (std::is_base_of<T2,T1>::value);
};


namespace rtlogger {
class SpdLogLogger: public LoggerCore {
public:
    SpdLogLogger() {
        logger = std::shared_ptr < spdlog::logger > (new spdlog::logger("multi_sink"));
        logger->set_level(spdlog::level::trace); // 全sinkにかかる、ログレベル
    }

    void set_pattern(const std::string& pattern){
        logger->set_pattern(pattern);// 全sinkにかかる、ログパターン
    }


    template<class T>
    void addSink(std::shared_ptr<T> sink,typename std::enable_if<is_derived_from<T,sinks::spdlog_sink_base_mt>::value>::type* = nullptr){
        logger->sinks().push_back(sink);
        sinks.push_back(sink);
    }

    template<class T>
    void addSink(std::shared_ptr<T> sink,typename std::disable_if<is_derived_from<T,sinks::spdlog_sink_base_mt>::value>::type* = nullptr){
        logger->sinks().push_back(sink);
    }

    ~SpdLogLogger() {
        spdlog::shutdown();
    }

    void setOutputLogLevel(LOG_LEVEL level) {
        spdlog::set_level(spdlog::level::info);
    }

    void putLog(const LogFormat &fmt) override {
        if (!logger) {
            return;
        }

        spdlog::level::level_enum level = spdlog::level::level_enum::trace;
        switch (static_cast<LOG_LEVEL>(fmt.level)) {
        case LOG_LEVEL::TRACE:
            level = spdlog::level::level_enum::trace;
            break;
        case LOG_LEVEL::DEBUG:
            level = spdlog::level::level_enum::debug;
            break;
        case LOG_LEVEL::INFO:
            level = spdlog::level::level_enum::info;
            break;
        case LOG_LEVEL::WARN:
            level = spdlog::level::level_enum::warn;
            break;
        case LOG_LEVEL::ERROR:
            level = spdlog::level::level_enum::err;
            break;
        case LOG_LEVEL::FATAL:
            level = spdlog::level::level_enum::critical;
            break;
        default:
            break;
        }

        for (auto &sink : sinks) {
            sink->set_format(fmt);
        }
        std::stringstream ss;
        ss<<"[tid:"<<fmt.tid<<"] "<<fmt.msg;
        logger->log(fmt.time, spdlog::source_loc { fmt.file, fmt.line, fmt.func }, level, ss.str());
    }

private:
    std::shared_ptr<spdlog::logger> logger = nullptr;
    std::vector<std::shared_ptr<sinks::spdlog_sink_base_mt>> sinks;
};
}
