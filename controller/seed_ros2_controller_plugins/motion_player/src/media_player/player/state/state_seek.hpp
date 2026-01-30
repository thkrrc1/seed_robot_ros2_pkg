#pragma once

#include "audio_player.hpp"
#include "video_player.hpp"

#include "state.hpp"

namespace motion_editor2 {

class StateSeek: public State {
private:
    bool acceptable(DECODING_STATE state) override{
        if (state == STATE_SEEK) {
            return true;
        }
        return false;
    }

    void entry(int step,double dt, MediaList &media_list, AudioPlayer &audio_player, VideoPlayer &video_player) override {
        audio_player.pause();
        video_player.close_window();
    }

    DECODING_STATE exec(int step,double dt, MediaList &media_list, AudioPlayer &audio_player, VideoPlayer &video_player) override{
        std::lock_guard<std::mutex> lock(media_list.mtx);
        seek_impl(step, dt, media_list.data);

        std::vector < MediaInfo2::Ptr > cur_media;
        getCurrentItems(step, cur_media,media_list.data);
        video_player.set_mediainfo(cur_media);
        video_player.play_until(step);

        return STATE_PAUSE;
    }

    std::string getname() override{
        return "seek";
    }

    void seek_impl(int step, double dt, std::list<MediaInfo2::Ptr> &media_list) {
        for (auto &media : media_list) {
            auto decoder = media->getDecoder();
            if (decoder) {
                double media_sec = media->st_sec;
                if (is_cur_item(step, media)) {
                    media_sec = media->st_sec + (step - media->step) * dt;
                }
                decoder->seek(media_sec);
            }
        }
    }

    bool is_cur_item(int step, const MediaInfo2::Ptr &item) {
        if (item->step <= step && item->step + item->dur_step >= step) {
            return true;
        }
        return false;
    }

    void getCurrentItems(int step, std::vector<MediaInfo2::Ptr> &info, std::list<MediaInfo2::Ptr> &media_list) {
        info.clear();
        auto itr = media_list.begin();
        while (itr != media_list.end() && (*itr)->step <= step) {
            if ((*itr)->step + (*itr)->dur_step >= step) {
                info.push_back(*itr);
            }
            ++itr;
        }
    }


};

}
