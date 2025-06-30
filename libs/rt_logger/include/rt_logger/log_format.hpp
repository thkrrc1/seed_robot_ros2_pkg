#pragma once

#if __has_include("archive.hpp")
#include "archive.hpp"
#endif

#include <chrono>

namespace rtlogger {
constexpr int CHAR_LEN_MAX = 256;
constexpr int LOG_STR_MAX = 4096;

enum class LOG_LEVEL : int {
    TRACE, DEBUG, INFO, WARN, ERROR, FATAL,
};

struct LogFormat {
    char pkg[CHAR_LEN_MAX] = { 0 };              //!< パッケージ名
    char func[CHAR_LEN_MAX] = { 0 };             //!< 関数名
    char file[CHAR_LEN_MAX] = { 0 };             //!< ファイルパス
    char pname[CHAR_LEN_MAX] = { 0 };            //!< プロセス名
    int line = 0;                                //!< 行数
    unsigned int pid = 0;                        //!< プロセスID
    unsigned int tid = 0;                        //!< スレッドID
    int level = 0;                               //!< ログレベル
    char msg[LOG_STR_MAX] = { 0 };               //!< ログメッセージ
    std::chrono::system_clock::time_point time;  //!< ログ出力時刻

#ifdef ARCHIVE_NAMEDVALUE
    friend class access;
    template<class T>
    void serialize(T &ar) {
        ar & ARCHIVE_NAMEDVALUE(pkg);
        ar & ARCHIVE_NAMEDVALUE(func);
        ar & ARCHIVE_NAMEDVALUE(file);
        ar & ARCHIVE_NAMEDVALUE(pname);
        ar & ARCHIVE_NAMEDVALUE(line);
        ar & ARCHIVE_NAMEDVALUE(pid);
        ar & ARCHIVE_NAMEDVALUE(tid);
        ar & ARCHIVE_NAMEDVALUE(level);
        ar & ARCHIVE_NAMEDVALUE(msg);
        ar & ARCHIVE_NAMEDVALUE(time);
    }
#endif
};
}
