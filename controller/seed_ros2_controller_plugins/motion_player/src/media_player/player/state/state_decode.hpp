#pragma once

#include "audio_player.hpp"
#include "video_player.hpp"

#include "state.hpp"

namespace motion_editor2 {

class StateDecode : public State{
private:
    bool acceptable(DECODING_STATE type) override{
        return true;
    }

    void entry(int step,double dt, MediaList &media_list, AudioPlayer &audio_player, VideoPlayer &video_player) override {
        std::lock_guard < std::mutex > lock(media_list.mtx);
        bool has_video = false;
        for (auto &data : media_list.data) {
            if (data->getVideoDecoder()) {
                has_video = true;
            }
        }

        audio_player.resume();

        if (has_video) {
            video_player.open_window(1280, 720);
        }
    }

    DECODING_STATE exec(int step,double dt, MediaList &media_list, AudioPlayer &audio_player, VideoPlayer &video_player) override {

        std::lock_guard<std::mutex> lock(media_list.mtx);
        decode_impl(step,media_list.data);

        if (prev_step != step) {
            std::vector < MediaInfo2::Ptr > cur_media;
            getCurrentItems(step, cur_media,media_list.data);
            audio_player.set_mediainfo(cur_media);
            video_player.set_mediainfo(cur_media);
            audio_player.play_until(step);
            video_player.play_until(step);
            prev_step = step;
        }


        return STATE_DECODE;
    }

    std::string getname() override{
        return "decode";
    }

    void decode_impl(int step, std::list<MediaInfo2::Ptr> &media_list){
        for (auto &media : media_list) {
            if (is_item_till(step, 100, media)) {
                auto decoder = media->getDecoder();
                if (decoder) {
                    decoder->decode_once();
                }
            }
        }
    }

    bool is_item_till(int step,int foresee_step,const MediaInfo2::Ptr& item){
        if (item->step <= step + foresee_step && item->step + item->dur_step >= step) {
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


private:
    int prev_step = 0;
};

}
