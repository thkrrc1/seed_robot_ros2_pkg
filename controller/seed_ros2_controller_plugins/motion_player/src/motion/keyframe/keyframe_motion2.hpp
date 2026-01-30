#pragma once

#include "common.hpp"
#include "motion_base.hpp"
#include "fileio_json.hpp"

#include "joint_state.hpp"
#include "keyframe_info.hpp"
#include "media_info.hpp"

namespace motion_editor2{
class KeyframeMotion2 : public MotionBase2{
public:
    KeyframeMotion2(){
    }

    void setStepTime(double dt){
        this->dt = dt;
    }

    void createDecoder(const std::string& wdir){
        for(auto &item : video){
            item.createDecoder(wdir);
        }

        for(auto &item : audio){
            item.createDecoder(wdir);
        }
    }

    void deleteDecoder(){
        for(auto &item : video){
            item.deleteDecoder();
        }

        for(auto &item : audio){
            item.deleteDecoder();
        }
    }


    double getStepTime() const{
        return dt;
    }

    std::vector<std::string> getJointNames() const{
        return jnames;
    }

    bool setJointNames(const std::vector<std::string> &jnames){
        //キーフレームが空の場合のみ変更可能
        if(keyframe.size() == 0){
            this->jnames = jnames;
            return true;
        }
        return false;
    }

    void load(const std::string& file_path){
         std::ifstream rfile;
         rfile.open(file_path);
         if(!rfile){
             LOG_INFO_STREAM()<<"cannot open file : "<<file_path<<LOG_END;
             return;
         }
         std::string str;
         std::string line;
         while (getline(rfile, line)) {
             str += line;
         }

         JsonInputArchive ar(str);
         ar >> *this;
     }

     void save(const std::string& file_path) const{
         std::ofstream wfile;
         wfile.open(file_path, std::ios::out);
         if(!wfile){
             LOG_INFO_STREAM()<<"cannot open file : "<<file_path<<LOG_END;
             return;
         }

         JsonOutputArchive ar(false);
         ar << *this;
         wfile << ar.commit();
     }


     std::list<AudioInfo2> getAudioList() override {
        return audio;
    }

    std::list<VideoInfo2> getVideoList() override {
        return video;
    }


private:
     MotionIterator2::Ptr getIterator() const override;

protected:
    friend class ::access;
    template<class T>
    void serialize(T &ar) {
        ar & ARCHIVE_NAMEDVALUE(dt);
        ar & ARCHIVE_NAMEDVALUE(jnames);
        ar & ARCHIVE_NAMEDVALUE(keyframe);
        ar & ARCHIVE_NAMEDVALUE(video);
        ar & ARCHIVE_NAMEDVALUE(audio);
    }

private:
    friend class KeyframeMotionIterator2;
    friend class KeyframeCommand;
    friend class KeyframeEditorCore;
    friend class AudioclipCommand;
    friend class AudioclipEditorCore;
    friend class VideoclipCommand;
    friend class VideoclipEditorCore;
    double dt = 0.0;//!< ステップ時間[s]
    std::vector<std::string> jnames;
    std::list<KeyFrameInfo2> keyframe;
    std::list<VideoInfo2> video;
    std::list<AudioInfo2> audio;
};
}
