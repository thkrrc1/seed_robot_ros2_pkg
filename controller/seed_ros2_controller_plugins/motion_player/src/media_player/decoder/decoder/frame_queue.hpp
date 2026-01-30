#pragma once

#include "avframe2.hpp"

namespace motion_editor2 {

class FrameQueue{
public:
    FrameQueue(){}
    virtual ~FrameQueue(){}

    void push(const AVFrame2::Ptr &frame) {
        std::lock_guard<std::mutex> lock(mtx);
        frames.push_back(frame);
    }

    AVFrame2::Ptr front(){
        std::lock_guard<std::mutex> lock(mtx);
        if (frames.empty()) {
            return nullptr;
        }
        auto frame = frames.front();
        return frame;
    }

    AVFrame2::Ptr pop() {
        std::lock_guard<std::mutex> lock(mtx);
        if (frames.empty()) {
            return nullptr;
        }
        auto frame = frames.front();
        frames.pop_front();
        return frame;
    }

    void clear(){
        std::lock_guard<std::mutex> lock(mtx);
        frames.clear();
    }

    int size(){
        std::lock_guard<std::mutex> lock(mtx);
        return frames.size();
    }

private:
    std::deque<AVFrame2::Ptr> frames;
    mutable std::mutex mtx;
};
}
