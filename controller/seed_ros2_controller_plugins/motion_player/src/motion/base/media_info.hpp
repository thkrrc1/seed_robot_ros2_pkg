#pragma once

#include "common.hpp"
#include "fileio.hpp"

#include "packet_handler.hpp"
#include "decoder_base.hpp"
#include "queue_sound_decoder.hpp"
#include "queue_video_decoder.hpp"

namespace motion_editor2{
struct MediaInfo2{
    using Ptr = MediaInfo2*;

    double st_sec = 0;  //!< メディアの再生開始場所
    double len_sec = 0; //!< メディアの全時間
    int step = 0;       //!< 再生開始ステップ
    int dur_step = 0;   //!< 再生ステップ数
    double volume = 1.0;  //!< 0~1;
    std::string fname;   //!< ファイル名
public:
    MediaInfo2(){}
    virtual ~MediaInfo2(){}

    virtual PacketHandler::Ptr getDecoder(){
        return nullptr;
    }

    virtual QueueSoundDecoder::Ptr getAudioDecoder(){
        return nullptr;
    }

    virtual QueueVideoDecoder::Ptr getVideoDecoder(){
        return nullptr;
    }

protected:
    friend class ::access;
    template<class T>
    void serialize(T &ar) {
        ar & ARCHIVE_NAMEDVALUE(st_sec);
        ar & ARCHIVE_NAMEDVALUE(len_sec);
        ar & ARCHIVE_NAMEDVALUE(step);
        ar & ARCHIVE_NAMEDVALUE(dur_step);
        ar & ARCHIVE_NAMEDVALUE(fname);
        ar & ARCHIVE_NAMEDVALUE(volume);
    }
};

struct VideoInfo2 : public MediaInfo2{

public:
    VideoInfo2(){}
    ~VideoInfo2(){}

    bool createDecoder(const std::string& wdir) {
        deleteDecoder();
        if (!fname.empty()) {
            decoder = std::make_shared<PacketHandler>();
            audio_decoder = std::make_shared<QueueSoundDecoder>();
            video_decoder = std::make_shared<QueueVideoDecoder>();
            if (decoder->load_file(wdir + "/" + fname) && decoder->set_decoder(audio_decoder, video_decoder)) {
                len_sec = decoder->get_dur_sec();
                return true;
            }
            deleteDecoder();
        }
        return false;
    }

    void deleteDecoder(){
        video_decoder = nullptr;
        audio_decoder = nullptr;
        decoder = nullptr;
    }

    PacketHandler::Ptr getDecoder() override{
        return decoder;
    }

    QueueSoundDecoder::Ptr getAudioDecoder() override{
        return audio_decoder;
    }

    QueueVideoDecoder::Ptr getVideoDecoder() override{
        return video_decoder;
    }

protected:
    friend class ::access;
    template<class T>
    void serialize(T &ar) {
        MediaInfo2::serialize(ar);
    }

private:
    PacketHandler::Ptr decoder = nullptr;
    QueueSoundDecoder::Ptr audio_decoder = nullptr;
    QueueVideoDecoder::Ptr video_decoder = nullptr;
};

struct AudioInfo2 : public MediaInfo2{
public:
    AudioInfo2(){}
    ~AudioInfo2(){}

    bool createDecoder(const std::string& wdir) {
        deleteDecoder();
        if (!fname.empty()) {
            decoder = std::make_shared<PacketHandler>();
            audio_decoder = std::make_shared<QueueSoundDecoder>();
            if (decoder->load_file(wdir + "/" + fname) && decoder->set_decoder(audio_decoder, nullptr)) {
                len_sec = decoder->get_dur_sec();
                return true;
            }
            deleteDecoder();
        }
        return false;
    }

    PacketHandler::Ptr getDecoder() override{
        return decoder;
    }

    QueueSoundDecoder::Ptr getAudioDecoder() override{
        return audio_decoder;
    }

    void deleteDecoder(){
        audio_decoder = nullptr;
        decoder = nullptr;
    }

protected:
    friend class ::access;
    template<class T>
    void serialize(T &ar) {
        MediaInfo2::serialize(ar);
    }
private:
    PacketHandler::Ptr decoder = nullptr;
    QueueSoundDecoder::Ptr audio_decoder = nullptr;
};
}
