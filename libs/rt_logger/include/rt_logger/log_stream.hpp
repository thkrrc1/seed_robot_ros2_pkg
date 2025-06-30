#pragma once

#include <cstring>
#include "log_format.hpp"
#include "log_sink_thread.hpp"
#include "singleton.hpp"
#include <streambuf>
#include <unistd.h>
#include <sys/syscall.h>

namespace rtlogger {

class LogStreamBuff: public std::basic_streambuf<char, std::char_traits<char> > {
    using Base = std::basic_streambuf<char, std::char_traits<char>>;
public:

    inline LogStreamBuff(LogFormat *fmt) :
            fmt(fmt) {
        cur_pos = fmt->msg;
    }

protected:

    int overflow(Base::int_type c = Base::traits_type::eof()) override
    {
        if (c != Base::traits_type::eof()) {
            if (cur_pos - fmt->msg + 2 < LOG_STR_MAX) {//最後のNULL文字も含めて、サイズチェック
                *cur_pos = c;
                ++cur_pos;
                return c;
            }
        }

        return Base::traits_type::eof();
    }

    int sync(void) override
    {
        cur_pos = fmt->msg;

        //最後の改行コードを消しておく
        auto len = strlen(fmt->msg);
        if (len > 0) {
            fmt->msg[len - 1] = '\0';
        }
        Singleton<LogSinkThread>::getInstance().insertLogFromStream(*fmt);
        fmt->msg[0] = '\0';
        return 0;
    }

private:
    LogFormat *fmt = nullptr;
    char *cur_pos = nullptr;
};

/**
 * delim区切りでの、最後の要素を取得
 */
inline const char* getLastSection(const char *full_text, char delim) {
    const char *last_section = full_text + std::strlen(full_text); //null文字位置
    while (last_section != full_text) {
        last_section--;
        if (*last_section == delim) {
            last_section++;
            break;
        }
    }
    return last_section;
}

class RealtimeLogStream: public std::basic_ostream<char, std::char_traits<char> > {
public:
    RealtimeLogStream(LOG_LEVEL level,const char *package, const char *file, int line, const char *function) :
            std::basic_ostream<char, std::char_traits<char> >(&buff), buff(&fmt) {
        fmt.level = static_cast<int>(level);
        std::strcpy(fmt.func, function);
        std::strcpy(fmt.file, getLastSection(file,'/'));
        std::strcpy(fmt.pkg, package);
        fmt.line = line;
        fmt.time = std::chrono::system_clock::now();
        fmt.tid = static_cast<unsigned int>(::syscall(SYS_gettid));
        fmt.pid = static_cast<unsigned int>(::getpid());
    }

private:
    LogFormat fmt;
    LogStreamBuff buff;
};
}
