#pragma once

#include <iomanip>
#include "log_stream.hpp"
#include "log_format.hpp"

#if __has_include("version_config.hpp")
#include "version_config.hpp"
#endif
#ifndef PACKAGE
#define PACKAGE ""
#endif

#define LOG_STREAM(level)\
        rtlogger::RealtimeLogStream(level,PACKAGE,__FILE__,__LINE__,__FUNCTION__)

#define LOG_TRACE_STREAM()\
        LOG_STREAM(rtlogger::LOG_LEVEL::TRACE)
#define LOG_DEBUG_STREAM()\
        LOG_STREAM(rtlogger::LOG_LEVEL::DEBUG)
#define LOG_INFO_STREAM()\
        LOG_STREAM(rtlogger::LOG_LEVEL::INFO)
#define LOG_WARN_STREAM()\
        LOG_STREAM(rtlogger::LOG_LEVEL::WARN)
#define LOG_ERROR_STREAM()\
        LOG_STREAM(rtlogger::LOG_LEVEL::ERROR)
#define LOG_FATAL_STREAM()\
        LOG_STREAM(rtlogger::LOG_LEVEL::FATAL)
#define LOG_END\
        std::endl

namespace rtlogger {
struct hexdump{
    hexdump(unsigned char *data, int len) :
            data(data), len(len) {
    }

    friend std::ostream& operator<<(std::ostream& os, const hexdump& dump);

private:
    unsigned char *data = nullptr;
    int len = 0;
};
inline std::ostream& operator<<(std::ostream &os, const hexdump &dump) {
    os << "\n";
    os << std::hex << std::setfill('0');
    for (int idx = 0; idx < dump.len; ++idx) {
        if (idx == 0) {
        } else if (idx % 32 == 0) {
            os << ("\n");
        } else if (idx % 2 == 0) {
            os << (" ");
        }
        os << std::setw(2)<< (int) dump.data[idx];
    }
    return os;
}
}
