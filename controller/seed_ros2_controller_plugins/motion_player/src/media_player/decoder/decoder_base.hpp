#pragma once

#include "common.hpp"

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

//#define HW_ACCEL

namespace motion_editor2 {

class DecoderBase{
public:
    using Ptr = std::shared_ptr<DecoderBase>;
#ifdef HW_ACCEL
    static enum AVPixelFormat get_hw_format(AVCodecContext *ctx,
                                            const enum AVPixelFormat *pix_fmts)
    {
        const enum AVPixelFormat *p;

        for (p = pix_fmts; *p != -1; p++) {
            if (*p == hw_pix_fmt)
                return *p;
        }

        fprintf(stderr, "Failed to get HW surface format.\n");
        return AV_PIX_FMT_NONE;
    }
#endif

    virtual ~DecoderBase(){
        close();
    }

    bool open_codec(AVStream* stream){
        this->stream = stream;

        codec = avcodec_find_decoder(stream->codecpar->codec_id);
        if (!codec) {
            close();
            return false;
        }

#ifdef HW_ACCEL
        enum AVHWDeviceType type = AV_HWDEVICE_TYPE_VDPAU;
        if(stream->codecpar->codec_type == AVMEDIA_TYPE_VIDEO){
//        AV_HWDEVICE_TYPE_NONE 
//        AV_HWDEVICE_TYPE_VDPAU 
//        AV_HWDEVICE_TYPE_CUDA 
//        AV_HWDEVICE_TYPE_VAAPI 
//        AV_HWDEVICE_TYPE_DXVA2 
//        AV_HWDEVICE_TYPE_QSV 
//        AV_HWDEVICE_TYPE_VIDEOTOOLBOX 
//        AV_HWDEVICE_TYPE_D3D11VA 
//        AV_HWDEVICE_TYPE_DRM 
//        AV_HWDEVICE_TYPE_OPENCL 
//        AV_HWDEVICE_TYPE_MEDIACODEC 
//        AV_HWDEVICE_TYPE_VULKAN 
        for (int i = 0;; i++) {
            const AVCodecHWConfig *config = avcodec_get_hw_config(codec, i);
            if (!config) {
                type = AV_HWDEVICE_TYPE_NONE;
                fprintf(stderr, "Decoder %s does not support device type %s.\n",
                        codec->name, av_hwdevice_get_type_name(type));
                return -1;
            }
            if (config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX &&
                config->device_type == type) {
                hw_pix_fmt = config->pix_fmt;
                break;
            }
        }
        }

#endif

        codec_context = avcodec_alloc_context3(codec);
        codec_context->thread_count = 8;
        codec_context->thread_type  = FF_THREAD_FRAME;
        if (!codec_context) {
            close();
            return false;
        }

        if (avcodec_parameters_to_context(codec_context, stream->codecpar) < 0) {
            close();
            return false;
        }

#ifdef HW_ACCEL
        if (stream->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            codec_context->get_format = get_hw_format;
            av_opt_set_int(codec_context, "refcounted_frames", 1, 0);
            av_hwdevice_ctx_create(&hw_device_ctx, type, NULL, NULL, 0);
            codec_context->hw_device_ctx = av_buffer_ref(hw_device_ctx);
        }
#endif

        if (avcodec_open2(codec_context, codec, nullptr) < 0) {
            close();
            return false;
        }

        if(!open_codec(stream,codec_context)){
            close();
            return false;
        }

        return true;
    }

    void flush_buffer(){
        if(codec_context){
            avcodec_flush_buffers(codec_context);
        }
    }

    double get_dur_sec(){
        if(!stream){
            return 0;
        }
        double dur_sec = static_cast<double>(stream->duration * stream->time_base.num) / stream->time_base.den;
        return dur_sec;
    }

    int get_stream_index() {
        if (stream) {
            return stream->index;
        } else {
            return -1;
        }
    }

    virtual double set_packet(const AVPacket *packet, bool seek = false) {
        return 0;
    }

    virtual void seek_start() {
    }

    virtual bool is_full(){
        return false;
    }

protected:
    virtual bool open_codec(AVStream* stream, AVCodecContext* codec_context){
        return false;
    }

private:
    void close(){
        codec = nullptr;
        stream = nullptr;
        if(codec_context){
            avcodec_free_context(&codec_context);
        }
    }

private:
    AVStream* stream = nullptr;
    const AVCodec * codec = nullptr;
    AVCodecContext* codec_context = nullptr;

#ifdef HW_ACCEL
    static AVBufferRef *hw_device_ctx;
    static enum AVPixelFormat hw_pix_fmt;
#endif
};

}
