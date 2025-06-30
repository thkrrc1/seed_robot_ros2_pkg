#pragma once

#include <spdlog/common.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/details/null_mutex.h>

#include <mutex>
#include <string>
#include <chrono>
#include <functional>
#include <iostream>

#include "../../log_format.hpp"

namespace rtlogger {
namespace sinks {

template<typename Mutex>
class spdlog_sink_base: public spdlog::sinks::base_sink<Mutex> {
public:
    explicit spdlog_sink_base() {
    }

    virtual ~spdlog_sink_base() override = default;

    virtual void set_format(const LogFormat &format) {
    }

};

using spdlog_sink_base_mt = spdlog_sink_base<std::mutex>;
using spdlog_sink_base_st = spdlog_sink_base<spdlog::details::null_mutex>;
}
}
