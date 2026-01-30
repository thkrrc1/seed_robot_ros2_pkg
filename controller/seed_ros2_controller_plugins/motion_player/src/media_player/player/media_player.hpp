#pragma once
#include <SDL2/SDL.h>

#include "audio_player.hpp"
#include "video_player.hpp"

#include "state_machine.hpp"

namespace motion_editor2 {

class MediaPlayer {
public:

    MediaPlayer() {
        if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0){
            std::cout << "Cound not initialize SDL2" << std::endl;
        }

        audio_player.open(44100,2);
        video_player.open(false);

        th_decode = std::thread(&MediaPlayer::decode_thread, this);
    }

    ~MediaPlayer() {
        shutdown();
        SDL_Quit();
    }

    void shutdown(){
        is_shutdown.store(true);
        if(th_decode.joinable()){
            th_decode.join();
        }
    }


    void setMediaList(const std::list<AudioInfo2> &audio_list, const std::list<VideoInfo2> &video_list) {
        stop();

        std::lock_guard<std::mutex> lock(media_list.mtx);
        media_list.data.clear();
        this->audio_list = audio_list;
        this->video_list = video_list;

        audio_player.set_mediainfo(std::vector<MediaInfo2::Ptr>());
        video_player.set_mediainfo(std::vector<MediaInfo2::Ptr>());

        for(auto &audio : this->audio_list){
            media_list.data.push_back(&audio);
        }
        auto itr = media_list.data.begin();
        for(auto &video : this->video_list){
            while(itr != media_list.data.end() && (*itr)->step <= video.step){
                ++itr;
            }
            media_list.data.insert(itr,&video);
        }
    }

    void set_step_time(double dt){
        this->dt = dt;
        video_player.set_steptime(dt);
        audio_player.set_steptime(dt);
    }

    void seek(int step,bool wait = false) {
        state_machine.send_request(STATE_SEEK, step);
        if(wait){
            state_machine.wait();
        }
    }

    void play_until(int step) {
        state_machine.send_request(STATE_DECODE,step);
    }

    void stop() {
        state_machine.send_request(STATE_PAUSE, 0);
    }

private:

    void decode_thread() {
        while (!is_shutdown.load()) {
            state_machine.exec(dt, media_list, audio_player, video_player);
            usleep(100);
        }
    }

private:
    std::thread th_decode;

    MediaList media_list;
    std::list<AudioInfo2> audio_list;
    std::list<VideoInfo2> video_list;

    AudioPlayer audio_player;
    VideoPlayer video_player;

    StateMachine state_machine;

    std::atomic<bool> is_shutdown = false;
    double dt = 0.02;
};

}
