#pragma once
#include <SDL2/SDL.h>

//#define PUBSUB_EXIST

#ifdef PUBSUB_EXIST
#include "pubsub.hpp"
#include "image_data.hpp"
#endif

namespace motion_editor2 {

class VideoPlayer{
public:


    ~VideoPlayer() {
        close_window();
    }

    void open(bool pubsub = true) {
        this->pubsub = pubsub;
    }

    void set_mediainfo(const std::vector < MediaInfo2::Ptr >& mediainfo){
        this->mediainfo = mediainfo;
    }

    void set_steptime(double dt){
        this->dt = dt;
    }

    void play_until(int step) {

        if (!pubsub) {
            //イベントを拾っておかないと、ウインドウサイズの変更が反映できない。
            SDL_Event e;
            while (SDL_PollEvent(&e)){}
        }

        for(auto &media : mediainfo){
            auto video_decoder = media->getVideoDecoder();
            if(!video_decoder){
                continue;
            }

            double media_sec = (step - media->step) * dt + media->st_sec;

            if (pubsub) {
#ifdef PUBSUB_EXIST
                ImageData img;
                if (!video_decoder->get_qpixmap(media_sec, img.image)) {
                    continue;
                }
                pubsub::Publisher<ImageData> pub("/images/image1", pubsub::LOCAL);
                pub.publish(img);
#endif
            } else {

                auto frame = video_decoder->get_frame(media_sec);
                if (!frame) {
                    continue;
                }

                //テクスチャのサイズを画像のサイズに合わせる
                int texture_w = 0;
                int texture_h = 0;
                if (texture) {
                    SDL_QueryTexture(texture, nullptr, nullptr, &texture_w, &texture_h);
                }
                if(frame->data()->width != texture_w || frame->data()->height != texture_h){
                    close_sdl_texture();
                    open_sdl_texture(frame->data()->width,frame->data()->height);
                }

                if (texture && renderer) {
                    SDL_Rect rect{0,0,frame->data()->width,frame->data()->height};
                    void* pixptr = nullptr;
                    SDL_LockTexture(texture, &rect, &pixptr, &frame->data()->linesize[0]);
                    std::memcpy(pixptr,frame->data()->data[0],frame->data()->linesize[0]*frame->data()->height);
                    SDL_UnlockTexture(texture);

                    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
                    SDL_RenderClear(renderer);
                    SDL_RenderCopy(renderer, texture, nullptr, nullptr);
                    SDL_RenderPresent(renderer);//バッファスワップ
                }
            }

            break;
        }
    }

    void open_window(int width, int height){
        if(!pubsub){
            open_sdl_window(width,height);
        }
    }

    void close_window(){
        if(!pubsub){
            close_sdl_window();
        }
    }


private:
    void open_sdl_texture(int width,int height){
        if (!texture) {
            texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, width, height);
        }
    }

    void close_sdl_texture(){
        if (texture) {
            SDL_DestroyTexture(texture);
            texture = nullptr;
        }
    }

    void open_sdl_window(int width, int height) {
        if (!window) {
            window = SDL_CreateWindow("video",
                    SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                    width, height,
                    SDL_WINDOW_RESIZABLE | SDL_WINDOW_MAXIMIZED);
        }

        if (!renderer) {
            renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
        }
    }

    void close_sdl_window() {
        if (renderer) {
            SDL_DestroyRenderer(renderer);
            renderer = nullptr;
        }

        if (window) {
            SDL_DestroyWindow(window);
            window = nullptr;
        }

        close_sdl_texture();
    }

private:
    bool pubsub = true;

    double dt = 0.02;
    std::vector < MediaInfo2::Ptr > mediainfo;

    SDL_Window * window = nullptr;
    SDL_Renderer * renderer = nullptr;
    SDL_Texture *texture = nullptr;
};
}
