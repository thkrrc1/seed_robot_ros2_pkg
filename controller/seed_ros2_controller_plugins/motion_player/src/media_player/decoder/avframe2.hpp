#pragma once

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavdevice/avdevice.h>
#include <libavfilter/avfilter.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libavutil/avutil.h>
#include <libpostproc/postprocess.h>
#include <libswresample/swresample.h>
#include <libswscale/swscale.h>
#include <libavutil/frame.h>
#include <libavutil/imgutils.h>
}


namespace motion_editor2 {

class AVFrame2 {
public:
    using Ptr = std::shared_ptr<AVFrame2>;

    AVFrame2() {
        frame = av_frame_alloc();
    }

    AVFrame2(AVFrame *frame) {
        this->frame = av_frame_clone(frame);
    }

    ~AVFrame2() {
        if (frame) {
            av_frame_free(&frame);
        }
    }

    AVFrame2::Ptr clone(){
        return std::make_shared<AVFrame2>(frame);
    }

    AVFrame* data() {
        return frame;
    }

private:
    AVFrame *frame = nullptr;
};

}
