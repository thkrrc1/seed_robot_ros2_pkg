#pragma once
#include <SDL2/SDL.h>

namespace motion_editor2 {

class AudioPlayer{
public:

    void open(int freq, int channels){
        SDL_AudioSpec want, have;
        SDL_zero(want);
        SDL_zero(have);
        want.freq = freq;
        want.channels = channels;
        want.format = AUDIO_S16SYS;
        want.samples = 1024; //1フレームのサンプル数
        want.userdata = this;
        want.callback = AudioPlayer::audio_callback2;
        for (int idx = 0; idx < SDL_GetNumAudioDevices(0); ++idx) {
            auto dev_names = SDL_GetAudioDeviceName(idx, 0);
            std::cout << "AVAILABLE AUDIO DEVICE : " << dev_names << std::endl;
        }
        audio_dev = SDL_OpenAudioDevice(nullptr, 0, &want, &have, SDL_AUDIO_ALLOW_CHANNELS_CHANGE);
        if (audio_dev <= 0) {
            std::cout<<"open audio failed"<<std::endl;
            return;
        }
        SDL_PauseAudioDevice(audio_dev, 0);//一時停止を解除して、再生モードにする
    }

    void set_mediainfo(const std::vector < MediaInfo2::Ptr >& mediainfo){
        std::lock_guard<std::mutex> lk(mediainfo_mtx);
        this->mediainfo = mediainfo;
    }


    void set_steptime(double dt){
        this->dt = dt;
    }

    void play_until(int step) {
        this->step = step;
    }

    void pause(){
        SDL_PauseAudioDevice(audio_dev, 1);
    }

    void resume(){
        SDL_PauseAudioDevice(audio_dev, 0);
    }

private:

    void audio_callback(uint8_t *stream, int len) {
        std::lock_guard < std::mutex > lk(mediainfo_mtx);
        SDL_memset(stream, 0, len);

        int step = this->step;
        for (auto &media : mediainfo) {
            auto audio_decoder = media->getAudioDecoder();
            if (!audio_decoder) {
                continue;
            }

            double media_sec = (step - media->step) * dt + media->st_sec;
            auto buff = audio_decoder->get_buffer(len, media_sec);

            SDL_MixAudioFormat(stream, buff->buff, AUDIO_S16SYS, buff->size, static_cast<int>(SDL_MIX_MAXVOLUME * media->volume));
        }
    }

    static void audio_callback2(void *userdata, uint8_t *stream, int len) {
        AudioPlayer* player = static_cast<AudioPlayer*>(userdata);
        player->audio_callback(stream,len);
    }


private:
    std::mutex mediainfo_mtx;
    double dt = 0.02;
    int step = 0;
    std::vector < MediaInfo2::Ptr > mediainfo;

    SDL_AudioDeviceID audio_dev = -1;

};



}
