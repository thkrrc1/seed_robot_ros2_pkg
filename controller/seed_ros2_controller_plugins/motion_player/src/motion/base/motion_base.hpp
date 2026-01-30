#pragma once

#include "common.hpp"
#include "iterator_base.hpp"
#include "media_info.hpp"

namespace motion_editor2 {
class MotionBase2 {
public:
    std::string name; //モーション名
    virtual ~MotionBase2() {
    }

    virtual MotionIterator2::Ptr getIterator() const = 0;

    virtual std::list<AudioInfo2> getAudioList() {
        return std::list<AudioInfo2>();
    }

    virtual std::list<VideoInfo2> getVideoList() {
        return std::list<VideoInfo2>();
    }

};
}
