#pragma once

#include "common.hpp"
#include "decoder_base.hpp"

namespace motion_editor2 {

class PacketHandler{
public:
    using Ptr = std::shared_ptr<PacketHandler>;

    PacketHandler(){
        packet = av_packet_alloc();
    }

    ~PacketHandler(){
        std::lock_guard<std::mutex> lk(decode_mtx);
        close();
        av_packet_free(&packet);
    }

    bool decode_once(){
        std::lock_guard<std::mutex> lk(decode_mtx);

        seeked_pts = -1;

        if(!format || codec_list.size() == 0){
            return false;
        }

        if(is_full()){
            return true;
        }

        if(av_read_frame(format, packet) >= 0){
            for(auto &codec : codec_list){
                if(packet->stream_index == codec->get_stream_index()){
                    codec->set_packet(packet);
                }
            }
            av_packet_unref(packet);
            return true;
        }
        return false;
    }

    void decode_all(){
        while (1) {
            if(!decode_once()){
                break;
            }
        }
    }

    bool seek(double sec){
        std::lock_guard<std::mutex> lk(decode_mtx);
        if(!format || codec_list.size() == 0){
            return false;
        }

        int64_t tgt_pts = static_cast<int64_t>(AV_TIME_BASE * sec);
        if(seeked_pts == tgt_pts){
            return true;
        }
        seeked_pts = tgt_pts;

        if (av_seek_frame(format, -1, tgt_pts, AVSEEK_FLAG_BACKWARD) < 0) {
            return false;
        }

        //シーク結果を反映
        for (auto &codec : codec_list) {
            codec->flush_buffer();
            codec->seek_start();
        }

        //目標のptsまで読み飛ばす
        std::vector<int> seeked_stream;
        while (av_read_frame(format, packet) >= 0) {
            for (auto &codec : codec_list) {
                if (packet->stream_index == codec->get_stream_index()) {
                    double cur_sec = codec->set_packet(packet, true);
                    if (cur_sec >= sec && std::find(seeked_stream.begin(), seeked_stream.end(), packet->stream_index) == seeked_stream.end()) {
                        seeked_stream.push_back(packet->stream_index);
                    }
                    break;
                }
            }

            bool fin = false;
            if(seeked_stream.size() == codec_list.size()){
                fin = true;
            }

            av_packet_unref(packet);

            if(fin){
                break;
            }
        }

        return true;
    }



    double get_dur_sec(){
        std::lock_guard<std::mutex> lk(decode_mtx);
        if(!format){
            return 0;
        }
        return static_cast<double>(format->duration) / AV_TIME_BASE;
    }

    bool load_file(const std::string &path) {
        std::lock_guard<std::mutex> lk(decode_mtx);
        close();
        format = avformat_alloc_context();
        if (avformat_open_input(&format, path.c_str(), nullptr, nullptr) != 0 || avformat_find_stream_info(format, nullptr) < 0) {
            close();
            return false;
        }
        return true;
    }

    bool set_decoder(DecoderBase::Ptr audio_decoder, DecoderBase::Ptr video_decoder){
        std::lock_guard<std::mutex> lk(decode_mtx);
        if(!format){
            return false;
        }

        codec_list.clear();

        for (size_t idx = 0; idx < format->nb_streams; idx++) {
            DecoderBase::Ptr decoder = nullptr;
            auto type = format->streams[idx]->codecpar->codec_type;

            if (type == AVMEDIA_TYPE_AUDIO) {
                decoder = audio_decoder;
            } else if (type == AVMEDIA_TYPE_VIDEO) {
                decoder = video_decoder;
            }

            if (decoder
                    && std::find(codec_list.begin(),codec_list.end(),decoder) == codec_list.end()
                    && decoder->open_codec(format->streams[idx])) {
                codec_list.push_back(decoder);
            }
        }

        return true;
    }

private:
    void close(){
        codec_list.clear();
        av_packet_unref(packet);

        if (format) {
            avformat_free_context(format);
            format = nullptr;
        }
    }

    bool is_full() {
        for (auto &codec : codec_list) {
            if (codec->is_full()) {
                return true;
            }
        }
        return false;
    }


private:
    std::mutex decode_mtx; //!< デコード作業用mutex
    AVFormatContext* format = nullptr;
    AVPacket *packet = nullptr;

    int64_t seeked_pts = -1;

    std::vector<DecoderBase::Ptr> codec_list;
};

}
