#pragma once

#include "common.hpp"
#include "decoder_base.hpp"
#include "avframe2.hpp"

namespace motion_editor2 {

class GetSoundWave: public DecoderBase {
public:
    using Ptr = std::shared_ptr<GetSoundWave>;
    using dtype = int16_t;

    GetSoundWave(int sample_rate, double dt_sec) :sample_rate(sample_rate),dt_sec(dt_sec){
    }

    ~GetSoundWave(){
        close();
    }

    void getData(std::vector<dtype> &data, dtype &peak) {
        std::lock_guard<std::mutex> lk(data_mtx);
        data = this->data;
        peak = this->peak;
    }

private:
    bool open_codec(AVStream* stream, AVCodecContext* codec_context) override{
        this->stream = stream;
        this->codec_context = codec_context;
        frame = std::make_shared<AVFrame2>();
        swr = swr_alloc();
        av_opt_set_int(swr, "in_channel_count", codec_context->channels, 0);
        av_opt_set_int(swr, "in_channel_layout", codec_context->channel_layout, 0);
        av_opt_set_int(swr, "in_sample_rate", codec_context->sample_rate, 0);
        av_opt_set_int(swr, "out_channel_count", 1, 0);
        av_opt_set_int(swr, "out_channel_layout", AV_CH_LAYOUT_MONO, 0);
        av_opt_set_int(swr, "out_sample_rate", sample_rate, 0);
        av_opt_set_sample_fmt(swr, "in_sample_fmt", codec_context->sample_fmt, 0);
        av_opt_set_sample_fmt(swr, "out_sample_fmt", format, 0);
        swr_init(swr);
        if (!swr_is_initialized(swr)) {
            close();
            return false;
        }
        return true;
    }

    double set_packet(const AVPacket *packet, bool seek) override{

        //デコーダにパケットを送る
        if (avcodec_send_packet(codec_context, packet) < 0) {
            return 0;
        }

        //この関数は最初にav_frame_unrefが呼ばれる点に注意
        while (avcodec_receive_frame(codec_context, frame->data()) == 0) {

            std::lock_guard<std::mutex> lk(data_mtx);

            dtype *buffer = nullptr;
            av_samples_alloc((uint8_t**) &buffer, nullptr, 1, frame->data()->nb_samples, format, 0);
            int frame_count = swr_convert(swr, (uint8_t**) &buffer, frame->data()->nb_samples, (const uint8_t**) frame->data()->data, frame->data()->nb_samples);

            double sec = static_cast<double>(frame->data()->pts * stream->time_base.num) / stream->time_base.den;
            for (int idx = 0; idx < frame_count; ++idx) {
                double sec2 = sec + static_cast<double>(idx) / sample_rate;
                if (sec2 > get_dur_sec()) {
                    break;
                }

                if (abs(buffer[idx]) > frame_max) {
                    frame_max = abs(buffer[idx]);
                }

                if (frame_max > peak) {
                    peak = frame_max;
                }

                if (sec2 - prev_sec >= dt_sec) {
                    data.push_back(frame_max);
                    prev_sec += dt_sec;
                    frame_max = 0;
                }
            }

            if (buffer) {
                av_freep(&buffer);
            }
        }
        return 0;
    }

    void close(){
        frame = nullptr;
        stream = nullptr;
        codec_context = nullptr;
        if(swr){
            swr_free(&swr);
        }
    }

private:
    struct SwrContext *swr = nullptr;
    AVFrame2::Ptr frame = nullptr;

    //入力
    int sample_rate = 0;
    double dt_sec = 0;

    //コンテキスト
    std::mutex data_mtx;
    double prev_sec = 0;
    dtype frame_max = 0;
    AVStream* stream = nullptr;
    AVCodecContext* codec_context = nullptr;

    AVSampleFormat format = AV_SAMPLE_FMT_S16;

    //出力
    std::vector<dtype> data;
    dtype peak = 0;

};

}
