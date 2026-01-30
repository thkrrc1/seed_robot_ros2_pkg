#pragma once
#include <SDL2/SDL.h>

#include "audio_player.hpp"
#include "video_player.hpp"

namespace motion_editor2 {

enum DECODING_STATE {
    STATE_PAUSE, STATE_SEEK, STATE_DECODE,
};

struct MediaList {
    std::mutex mtx;
    std::list<MediaInfo2::Ptr> data;
};

class State {
public:
    virtual ~ State() {
    }
    virtual DECODING_STATE exec(int step, double dt, MediaList &media_list, AudioPlayer &audio_player, VideoPlayer &video_player) {
        return STATE_PAUSE;
    }

    virtual void entry(int step,double dt, MediaList &media_list, AudioPlayer &audio_player, VideoPlayer &video_player) {
    }

    virtual void exit(int step,double dt, MediaList &media_list, AudioPlayer &audio_player, VideoPlayer &video_player) {
    }

    virtual bool acceptable(DECODING_STATE state) {
        return false;
    }

    virtual std::string getname() {
        return "";
    }
};

}
