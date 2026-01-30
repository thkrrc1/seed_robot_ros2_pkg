#pragma once

#include "common.hpp"
#include "decoder_base.hpp"
#include "avframe2.hpp"
#include "frame_queue.hpp"

namespace motion_editor2 {

struct AudioBuff{
public:
    using Ptr = std::shared_ptr<AudioBuff>;

    AudioBuff(int max_size){
        buff = new uint8_t[max_size];
        memset(buff,0x00,max_size);
    }

    ~AudioBuff(){
        delete buff;
    }

    int size = 0;
    uint8_t* buff = nullptr;
};

class QueueSoundDecoder: public DecoderBase {
public:
    using Ptr = std::shared_ptr<QueueSoundDecoder>;

    QueueSoundDecoder(){
    }

    ~QueueSoundDecoder(){
        close();
    }

    AudioBuff::Ptr get_buffer(int size_expect, double sec_until = -1){
        AudioBuff::Ptr buff = std::make_shared<AudioBuff>(size_expect);

        int offset = prev_offset;
        int size_remain = size_expect;
        auto frame = queue.front();
        auto cur_buff = buff->buff;
        int64_t bytes_per_sec = av_get_channel_layout_nb_channels(channel_layout) * av_get_bytes_per_sample(format) * sample_rate;
        while(frame){
            int64_t max_size = frame->data()->nb_samples * frame->data()->channels * av_get_bytes_per_sample(format);
            int copy_size = max_size - offset;

            //コピーサイズが、sec_untilを超えないようにする
            double frame_sec = static_cast<double>(frame->data()->pts * stream->time_base.num) / stream->time_base.den;
            double frame_end_sec = frame_sec + static_cast<double>(copy_size) / bytes_per_sec;
            if(frame_end_sec > sec_until){
                copy_size = static_cast<int>(std::floor((sec_until - frame_sec) * bytes_per_sec));
            }

            //残りバイト数以内に収める
            if(size_remain < copy_size){
                copy_size = size_remain;
            }

            if (copy_size <= 0) {
                break;
            }

            memcpy(cur_buff, frame->data()->data[0] + offset, copy_size);
            buff->size += copy_size;
            offset += copy_size;
            cur_buff += copy_size;
            size_remain -= copy_size;

            if(offset >= max_size){
                queue.pop();
                frame = queue.front();
                offset = 0;
            }
        }

        prev_offset = offset;

        return buff;
    }

    bool is_full() override{
        return queue.size() > 100;
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
        av_opt_set_sample_fmt(swr, "in_sample_fmt", codec_context->sample_fmt, 0);
        av_opt_set_int(swr, "out_channel_count", av_get_channel_layout_nb_channels(channel_layout), 0);
        av_opt_set_int(swr, "out_channel_layout", channel_layout, 0);
        av_opt_set_int(swr, "out_sample_rate", sample_rate, 0);
        av_opt_set_sample_fmt(swr, "out_sample_fmt", format, 0);
        swr_init(swr);
        if (!swr_is_initialized(swr)) {
            close();
            return false;
        }
        return true;
    }

    void seek_start() override{
        queue.clear();
        prev_offset = 0;
    }

    double set_packet(const AVPacket *packet, bool seek) override{

        double sec = 0;
        //デコーダにパケットを送る
        if (avcodec_send_packet(codec_context, packet) < 0) {
            return sec;
        }

        //この関数は最初にav_frame_unrefが呼ばれる点に注意
        while (avcodec_receive_frame(codec_context, frame->data()) == 0) {
                int nb_samples = swr_get_out_samples(swr, frame->data()->nb_samples);

                AVFrame2::Ptr frame2 = std::make_shared<AVFrame2>();
                av_frame_copy_props(frame2->data(),frame->data());
                frame2->data()->channel_layout = channel_layout;
                frame2->data()->nb_samples = nb_samples;
                frame2->data()->format = format;
                av_frame_get_buffer(frame2->data(), 0);
                frame2->data()->nb_samples = swr_convert(swr, frame2->data()->data, frame2->data()->nb_samples,
                        (const uint8_t**) frame->data()->data, frame->data()->nb_samples);

            if (seek && is_full()) {//seek中にいっぱいになった場合は、先頭を削除
                queue.pop();
            }
            queue.push(frame2);
            sec = static_cast<double>(frame->data()->pts * stream->time_base.num) / stream->time_base.den + static_cast<double>(frame->data()->nb_samples) / codec_context->sample_rate;
        }
        return sec;
    }

    void close(){
        frame = nullptr;
        stream = nullptr;
        codec_context = nullptr;
        if(swr){
            swr_free(&swr);
        }
        queue.clear();
    }

private:
    struct SwrContext *swr = nullptr;
    AVFrame2::Ptr frame = nullptr;

    //入力
    int sample_rate = 44100;
    AVSampleFormat format = AV_SAMPLE_FMT_S16;
    int64_t channel_layout =  AV_CH_LAYOUT_STEREO;


    //コンテキスト
    AVStream* stream = nullptr;
    AVCodecContext* codec_context = nullptr;

    //出力
    FrameQueue queue;
    int prev_offset = 0;
};

}
