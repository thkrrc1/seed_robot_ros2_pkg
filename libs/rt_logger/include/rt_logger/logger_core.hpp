#pragma once

#include "log_format.hpp"

namespace rtlogger {
class LoggerCore {
public:
    virtual ~LoggerCore() {
    }

    virtual void putLog(const LogFormat &fmt) {

    }
};
}
