#include "rt_logger/logger.hpp"
#include "rt_logger/spdlog_logger.hpp"
#include <unistd.h>
#include <iostream>

int main(int argc, char **argv) {
    LOGGER_INIT
    auto loggerCore = std::make_shared<rtlogger::SpdLogLogger>();
    rtlogger::setLoggerCore(loggerCore);

    LOG_TRACE_STREAM() << "severity is trace. " << std::string("string") << LOG_END;
    LOG_DEBUG_STREAM() << "severity is debug. " << "char array" << LOG_END;
    LOG_INFO_STREAM()  << "severity is info. "  << 3 << LOG_END;
    LOG_WARN_STREAM()  << "severity is warn. "  << 4 << LOG_END;
    LOG_ERROR_STREAM() << "severity is error. " << 5.0 << LOG_END;
    LOG_FATAL_STREAM() << "severity is fatal. " << 6.0 << LOG_END;
    return 0;
}
