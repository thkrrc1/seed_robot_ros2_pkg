#pragma once

#include "audio_player.hpp"
#include "video_player.hpp"

#include "state.hpp"

namespace motion_editor2 {

class StatePause : public State{
private:
    bool acceptable(DECODING_STATE state) override{
        return true;
    }

    void entry(int step,double dt, MediaList &media_list, AudioPlayer &audio_player, VideoPlayer &video_player) override {
        audio_player.pause();
        video_player.close_window();
    }

    void exit(int step,double dt, MediaList &media_list, AudioPlayer &audio_player, VideoPlayer &video_player) override {

    }

    DECODING_STATE exec(int step,double dt, MediaList &media_list, AudioPlayer &audio_player, VideoPlayer &video_player) override{
        usleep(1000);
        return STATE_PAUSE;
    }

    std::string getname() override{
        return "pause";
    }
};

}
