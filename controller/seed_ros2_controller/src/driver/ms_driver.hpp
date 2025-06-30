#pragma once

#include <yaml-cpp/yaml.h>
#include "aero_conversion.hpp"
#include "aero3_command.hpp"

class MSDriver {
public:
    MSDriver(const YAML::Node& settings) {

        if (!settings["msid"]) {
            return;
        }
        msid = settings["msid"].as<int>();

        std::vector<unsigned int> aero_index;
        if (settings["joints"]) {
            auto joints_params = settings["joints"];
            for (size_t idx = 0; idx < joints_params.size(); ++idx) {
                auto joint_params = joints_params[idx];
                if(!joint_params["name"] || !joint_params["aero_idx"]){
                    continue;
                }

                auto name = joint_params["name"].as<std::string>();
                auto aero_idx = joint_params["aero_idx"].as<unsigned int>();

                joint_names.push_back(name);
                aero_index.push_back(aero_idx);
            }
        }

        //通信順に並べる
        for (int idx = 0; idx < AXIS_MAX; ++idx) {
            auto itr = std::find(aero_index.begin(), aero_index.end(), idx);
            if (itr == aero_index.end()) {
                joint_names_message_order.push_back("");
            } else {
                size_t index = std::distance(aero_index.begin(), itr);
                joint_names_message_order.push_back(joint_names[index]);
            }
        }

        aero_conversion = new AeroConversion(aero_index);

        aero_strokes.resize(AXIS_MAX,0x7FFF);
        prev_strokes.resize(AXIS_MAX,0x7FFF);

        aero_velocities.resize(AXIS_MAX,0x7FFF);
        prev_velocities.resize(AXIS_MAX,0x7FFF);

        current_positions.resize(AXIS_MAX,0x0000);
        current_velocities.resize(AXIS_MAX,0x0000);
    }

    int getMsId(){
         return msid;
     }

    void getPosition(int16_t* ros_pos,AeroDriver *aero_driver){
        for(int idx = 0;idx < AXIS_MAX;++idx){
            auto pos = aero_driver->getpos(msid,idx);
            if(pos != 0x7FFF){
                current_positions[idx] = pos;
            }
        }
        aero_conversion->remapAeroToRos(ros_pos,current_positions);
    }

    void getMsStatus(uint16_t &status, AeroDriver *aero_driver) {
        status = aero_driver->getstatus(msid);
    }

//速度は取得できない
//    void getActualVelocities(std::vector<int16_t> &ros_velocities,AeroCommand *aero_driver){
//        for(int idx = 0;idx < 31;++idx){
//            if(aero_velocities[idx] != 0x7FFF){//移動に利用したものを、現在値として利用
//                current_velocities[idx] = aero_velocities[idx];
//            }
//        }
//
//        aero_conversion->remapAeroToRos(ros_velocities,current_velocities);
//    }

    int getNumJoints(){
        return joint_names.size();
    }

    //非RT
    std::string getJointName(int idx){
        size_t idx_tmp = static_cast<size_t>(idx);
        if(0<= idx_tmp && idx_tmp < joint_names.size()){
            return joint_names[idx_tmp];
        }
        return "";
    }

    //非RT
    /**
     * AeroIndexで並び替えられた関節名
     */
    std::vector<std::string> getJointNamesMessageOrder(){
        return joint_names_message_order;
    }

    //非RT
    std::vector<std::string> getJointNames(){
        return joint_names;
    }

    void sendPosition(const int16_t* ros_strokes, const double &tgt_time_sec,AeroDriver *aero_driver) {

        aero_conversion->remapRosToAero(aero_strokes,ros_strokes);
        size_t idxMax = aero_strokes.size() < prev_strokes.size() ? aero_strokes.size(): prev_strokes.size();

        bool exist_sendd = false;
        //前回送信値と同じであれば、送信しない。
        for (size_t idx = 0; idx < idxMax; ++idx) {
            if (aero_strokes[idx] == prev_strokes[idx]) {
                aero_strokes[idx] = 0x7FFF; // 値が同じであれば、送信しない。
            } else {
                prev_strokes[idx] = aero_strokes[idx];
                exist_sendd = true;
            }
        }

        //ロボットに送信
        if (exist_sendd) {
            has_sendd = true;
            aero_driver->sendMOVE(msid, tgt_time_sec, aero_strokes.data());
        }
    }

    void sendVelocity(const int16_t* ros_velocities,AeroDriver *aero_driver){
        aero_conversion->remapRosToAero(aero_velocities,ros_velocities);
        size_t idxMax = aero_velocities.size() < prev_velocities.size() ? aero_velocities.size(): prev_velocities.size();
        bool exist_sendd = false;
        //前回送信値と同じであれば、送信しない。
        for (size_t idx = 0; idx < idxMax; ++idx) {
            if (aero_velocities[idx] == prev_velocities[idx] && (aero_velocities[idx] == 0x0000 ||  aero_velocities[idx] == 0x7FFF)) {
                aero_velocities[idx] = 0x7FFF; // 値が同じであれば、送信しない。
            } else {
                prev_velocities[idx] = aero_velocities[idx];
                exist_sendd = true;
            }
        }

        //ロボットに送信
        if (exist_sendd) {
            has_sendd = true;
            aero_driver->sendTURN(msid, aero_velocities.data());
        }
    }

    bool getOtherCommands(BuffList &cmds, AeroDriver *aero_driver) {
        aero_driver->getOtherCommands(msid, cmds);
        return true;
    }


    bool sendOtherCommands(BuffList &cmds, AeroDriver *aero_driver) {
        if (cmds.size != 0) {
            has_sendd = true;
            aero_driver->sendOtherCommands(msid, cmds);
        }
        return true;
    }

    void sendIdleCommand(AeroDriver *aero_driver){
        //送信データがなければ、状態取得コマンドを発行する
        if (!has_sendd) {
            aero_driver->sendPGET(msid);
        }
        has_sendd = false;
    }

private:
    bool has_sendd = false;
    static const int AXIS_MAX = 31;

    int msid;
    AeroConversion *aero_conversion = nullptr;

    std::vector < std::string > joint_names_message_order;
    std::vector < std::string > joint_names;

    std::vector<int16_t> aero_velocities;
    std::vector<int16_t> prev_velocities;

    std::vector<int16_t> aero_strokes;
    std::vector<int16_t> prev_strokes;

    std::vector<int16_t> current_positions;
    std::vector<int16_t> current_velocities;

};
