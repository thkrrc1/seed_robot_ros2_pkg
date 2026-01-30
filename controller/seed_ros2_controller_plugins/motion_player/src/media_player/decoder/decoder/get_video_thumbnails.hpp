#pragma once

#include "ui_common.hpp"
#include "decoder_base.hpp"
#include "avframe2.hpp"

namespace motion_editor2 {

class GetVideoThumbnails: public DecoderBase {
public:
    using Ptr = std::shared_ptr<GetVideoThumbnails>;

    GetVideoThumbnails(double height, double dt_sec) :
            height(height), dt_sec(dt_sec) {
    }

    ~GetVideoThumbnails(){
        close();
    }

    std::vector<QImage> getData() {
        std::lock_guard<std::mutex> lk(data_mtx);
        return data;
    }

    double getImageWidth(){
        if(codec_context){
            return width;
        }
        return -1;
    }

private:
    bool open_codec(AVStream* stream, AVCodecContext* codec_context) override{
        this->stream = stream;
        this->codec_context = codec_context;

        width = codec_context->width * height / codec_context->height;
        conversion = sws_getContext(codec_context->width, codec_context->height, codec_context->pix_fmt,
                width, height, format,
                SWS_FAST_BILINEAR, NULL, NULL, NULL);

        frame_tmp = std::make_shared<AVFrame2>();
        frame_conv = std::make_shared<AVFrame2>();
        av_image_alloc(frame_conv->data()->data, frame_conv->data()->linesize, width, height, format, 32);

        return true;
    }

    double set_packet(const AVPacket *packet, bool seek) override{

        //デコーダにパケットを送る
        if (avcodec_send_packet(codec_context, packet) < 0) {
            return 0;
        }

        //この関数は最初にav_frame_unrefが呼ばれる点に注意
        while (avcodec_receive_frame(codec_context, frame_tmp->data()) == 0) {

            std::lock_guard<std::mutex> lk(data_mtx);

            if (codec_context->hwaccel != nullptr) {
                frame = std::make_shared<AVFrame2>();
                if (av_hwframe_transfer_data(frame->data(), frame_tmp->data(), 0) < 0) {
                    continue;
                }
                av_frame_copy_props(frame->data(), frame_tmp->data());
            } else {
                frame = frame_tmp;
            }

            double sec = static_cast<double>(frame->data()->pts * stream->time_base.num) / stream->time_base.den;

            if (sec - prev_sec >= dt_sec) {
                sws_scale(conversion, frame->data()->data, frame->data()->linesize, 0, frame->data()->height,
                        frame_conv->data()->data, frame_conv->data()->linesize);
                QImage image(frame_conv->data()->data[0], width, height, frame_conv->data()->linesize[0], QImage::Format_RGB888);
                data.push_back(image.copy());
                prev_sec += dt_sec;
            }
        }
        return 0;
    }

    void close(){
        frame = nullptr;
        frame_conv = nullptr;
        stream = nullptr;
        codec_context = nullptr;
        if(conversion){
            sws_freeContext(conversion);
            conversion = nullptr;
        }
    }

private:
    //コンテキスト
    std::mutex data_mtx;
    SwsContext *conversion = nullptr;
    AVFrame2::Ptr frame_tmp = nullptr;
    AVFrame2::Ptr frame = nullptr;
    AVFrame2::Ptr frame_conv = nullptr;
    double prev_sec = 0;
    AVStream* stream = nullptr;
    AVCodecContext* codec_context = nullptr;

    //入力
    double width = 0;
    double height = 0;
    double dt_sec = 0;
    AVPixelFormat format = AV_PIX_FMT_RGB24;

    //出力
    std::vector<QImage> data;
};

}
