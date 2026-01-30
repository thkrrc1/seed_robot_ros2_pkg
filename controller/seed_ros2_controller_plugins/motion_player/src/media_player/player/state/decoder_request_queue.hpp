#pragma once

#include "audio_player.hpp"
#include "video_player.hpp"
#include "state.hpp"

namespace motion_editor2 {

struct DecoderRequestInfo{
    DECODING_STATE state = STATE_PAUSE;
    bool processing = false;
    int step = 0;
};

class DecoderRequestQueue {
public:
    void add(const DecoderRequestInfo &info, bool guard = true) {
        if(guard){lock();}
        //同じ種別のリクエストはマージする
        if (req_queue.size() != 0 && !req_queue.back().processing && req_queue.back().state == info.state) {
            req_queue.pop_back();
        }
        req_queue.push_back(info);
        if(guard){unlock();}
    }

    size_t size(bool guard = true) {
        if(guard){lock();}
        size_t ret = req_queue.size();
        if(guard){unlock();}
        return ret;
    }

    void lock(){
        mtx.lock();
    }

    void unlock(){
        mtx.unlock();
    }

    /**
     * addで上書きされないようにする
     */
    void lock_front(bool guard = true) {
        if(guard){lock();}
        if (req_queue.size() != 0) {
            req_queue.front().processing = true;
        }
        if(guard){unlock();}
    }

    DecoderRequestInfo front(bool guard = true) {
        DecoderRequestInfo ret{ STATE_PAUSE, 0 };
        if(guard){lock();}
        if (req_queue.size() != 0) {
            ret = req_queue.front();
        }
        if(guard){unlock();}
        return ret;
    }

    void pop(bool guard = true) {
        if(guard){lock();}
        if (req_queue.size() != 0) {
            req_queue.pop_front();
        }
        if(guard){unlock();}
    }

private:
    std::mutex mtx;
    std::deque<DecoderRequestInfo> req_queue;
};

}
