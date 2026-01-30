#pragma once

#include <QPixmap>

#include "common.hpp"
#include "fileio.hpp"

/**
 * キーフレーム単体のデータ
 */
struct KeyFrameInfo2{
    int step = 0;          //<! 配置ステップ
    std::vector<JointState2> state;

protected:
    friend class ::access;
    template<class T>
    void serialize(T &ar) {
        ar & ARCHIVE_NAMEDVALUE(step);
        ar & ARCHIVE_NAMEDVALUE(state);
    }
};
