#pragma once

#include <mutex>
#include <string>
#include <chrono>
#include <functional>

#include "spdlog_sink_base.hpp"
#include "pubsub.hpp"

namespace rtlogger {
namespace sinks {

template<typename Mutex>
class pubsub_sink: public rtlogger::sinks::spdlog_sink_base<Mutex> {
public:
    explicit pubsub_sink(std::string topic, pubsub::SendType type = pubsub::SendType::GLOBAL) :
            pub(topic, type) {
    }

    ~pubsub_sink() override = default;

protected:
    void set_format(const LogFormat &format) override {
        this->format = format;
    }

    void sink_it_(const spdlog::details::log_msg &msg) override
    {
        pub.publish(format);
    }

    void flush_() override {
    }

private:
    pubsub::Publisher<LogFormat> pub;
    LogFormat format;
};

using pubsub_sink_mt = pubsub_sink<std::mutex>;
using pubsub_sink_st = pubsub_sink<spdlog::details::null_mutex>;
}
}
