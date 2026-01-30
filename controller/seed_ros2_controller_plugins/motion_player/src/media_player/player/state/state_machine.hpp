#pragma once

#include "audio_player.hpp"
#include "decoder_request_queue.hpp"
#include "video_player.hpp"

#include "state_pause.hpp"
#include "state_seek.hpp"
#include "state_decode.hpp"

namespace motion_editor2 {

class StateMachine {
public:
    StateMachine() {
        state_list.push_back(&state_pause);
        state_list.push_back(&state_seek);
        state_list.push_back(&state_decode);
    }

    void send_request(DECODING_STATE state, int step = 0) {
        DecoderRequestInfo req;
        req.state = state;
        req.step = step;
        request.add(req);
    }

    void wait() {
        while (request.size() != 0) {
            usleep(100000);
        }
    }

    void exec(double dt, MediaList &media_list, AudioPlayer &audio_player, VideoPlayer &video_player) {
        auto new_state = cur_state;
        bool pop = false;

        request.lock();
        if (request.size(false) != 0) {
            auto req = request.front(false);
            if (cur_state->acceptable(req.state)) {
                request.lock_front(false);
                new_state = state_list[req.state];
                step = req.step;
                pop = true;
            }
        }
        request.unlock();

        if (cur_state != new_state) {
            LOG_TRACE_STREAM() << "decode state changed (step: " << step << ") [" << cur_state->getname() << "] -> [" << new_state->getname() << "]" << LOG_END;
            cur_state->exit(step, dt, media_list, audio_player, video_player);
            new_state->entry(step, dt, media_list, audio_player, video_player);
            cur_state = new_state;
        }

        auto new_state_idx = cur_state->exec(step, dt, media_list, audio_player, video_player);
        new_state = state_list[new_state_idx];

        if (cur_state != new_state) {
            LOG_TRACE_STREAM() << "decode state changed (step: " << step << ") [" << cur_state->getname() << "] -> [" << new_state->getname() << "]" << LOG_END;
            cur_state->exit(step, dt, media_list, audio_player, video_player);
            new_state->entry(step, dt, media_list, audio_player, video_player);
            cur_state = new_state;
        }

        //waitのロックがリクエスト実行完了後に外れるようにする。
        if (pop) {
            request.pop();
        }
    }

private:
    int step = 0;

    DecoderRequestQueue request;
    StateSeek state_seek;
    StatePause state_pause;
    StateDecode state_decode;
    std::vector<State*> state_list;
    State *cur_state = &state_pause;
};

}
