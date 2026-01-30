#pragma once

#include "ui_common.hpp"
#include "decoder_base.hpp"
#include "avframe2.hpp"

namespace motion_editor2 {

class QueueVideoDecoder: public DecoderBase {
public:
    using Ptr = std::shared_ptr<QueueVideoDecoder>;

    QueueVideoDecoder() {
    }

    ~QueueVideoDecoder(){
        close();
    }

    AVFrame2::Ptr get_frame(double sec){
        AVFrame2::Ptr ret = nullptr;
        auto frame = queue.front();
        while(frame){
            double frame_sec = static_cast<double>(frame->data()->pts * stream->time_base.num) / stream->time_base.den;
            if(frame_sec <= sec){
                ret = queue.pop();
                frame = queue.front();
            }else{
                break;
            }
        }
        return ret;
    }

    bool get_qpixmap(double sec,QImage& image) {
        auto frame = get_frame(sec);
        if(frame){
            image = QImage(frame->data()->data[0], frame->data()->width, frame->data()->height, frame->data()->linesize[0], QImage::Format_RGB888).copy();
            return true;
        }
        return false;
    }

    bool is_full() override{
        return queue.size() > 10;
    }

private:
    bool open_codec(AVStream* stream, AVCodecContext* codec_context) override{
        this->stream = stream;
        this->codec_context = codec_context;
        frame = std::make_shared<AVFrame2>();

        width = codec_context->width;
        height = codec_context->height;
        format = AVPixelFormat::AV_PIX_FMT_RGB24;
        conversion = sws_getContext(codec_context->width, codec_context->height, codec_context->pix_fmt,
                width, height, format,
                SWS_FAST_BILINEAR, NULL, NULL, NULL);

        return true;
    }

    void seek_start() override{
        queue.clear();
    }

    double set_packet(const AVPacket *packet, bool seek) override{
        double sec = 0;
        //デコーダにパケットを送る
        if (avcodec_send_packet(codec_context, packet) < 0) {
            return sec;
        }

        //この関数は最初にav_frame_unrefが呼ばれる点に注意
        while (avcodec_receive_frame(codec_context, frame->data()) == 0) {

            AVFrame2::Ptr frame2 = std::make_shared<AVFrame2>();
            frame2->data()->pts = frame->data()->pts;
            frame2->data()->width = width;
            frame2->data()->height = height;
            frame2->data()->format = format;
            av_frame_get_buffer(frame2->data(), 32);

            sws_scale(conversion, frame->data()->data, frame->data()->linesize, 0, frame->data()->height,
                    frame2->data()->data, frame2->data()->linesize);

            if (is_full()) {
                queue.pop();
            }
            queue.push(frame2);

            sec = static_cast<double>(frame->data()->pts * stream->time_base.num) / stream->time_base.den;
        }
        return sec;
    }

    void close(){
        frame = nullptr;
        stream = nullptr;
        codec_context = nullptr;
        queue.clear();
        if(conversion){
            sws_freeContext(conversion);
            conversion = nullptr;
        }
    }

private:
    //コンテキスト
    AVFrame2::Ptr frame = nullptr;
    AVStream* stream = nullptr;
    AVCodecContext* codec_context = nullptr;
    SwsContext *conversion = nullptr;

    //入力
    int width = 1920;
    int height = 1080;
    AVPixelFormat format = AVPixelFormat::AV_PIX_FMT_RGB24;

    //出力
    FrameQueue queue;
};

}
