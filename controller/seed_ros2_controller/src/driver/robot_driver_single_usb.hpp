#pragma once

#include <seed_ros2_controller/command_interface/other_command/other_cmd_buff.hpp>
#include <seed_ros2_controller/command_interface/status/robot_status.hpp>

#include "ms_driver.hpp"
#include "rt_logger/logger.hpp"

#include "aero3_command.hpp"
#include "aero4_command.hpp"
#include <iostream>

class RobotDriverSingleUSB {
public:
    RobotDriverSingleUSB(std::string usb_port, int baudrate) :
            usb_port(usb_port), baudrate(baudrate) {
        aero_driver = new AeroDriver();
    }

    //非RT
    bool open() {
        if (aero_driver->openPort(usb_port, baudrate)) {
            LOG_INFO_STREAM() << "usb port was successfully opened. :" << usb_port << LOG_END;
            return true;
        } else {
            LOG_ERROR_STREAM() << "usb port was not opened. :" << usb_port << LOG_END;
            return false;
        }
    }

    //非RT
    void close() {
        aero_driver->closePort();
    }

    //非RT
    void setProtocol(const std::string &protocol){
        if(aero_command){
            delete aero_command;
            aero_command = nullptr;
        }

        if(protocol == "aero4"){
            aero_command = new aero4::AeroCommand();
            aero_driver->setCommand(aero_command);
            //バージョン取得　暫定処理
            aero_driver->setMsVersionCallback(
                [this](int msid, const std::string& ver_hex){
                    for (auto* ms : ms_drivers) {
                        if (ms && ms->getMsId() == msid) {
                            LOG_INFO_STREAM()<< " MSID : " << ms->getMsId() << ",  "<< "MS Firmware Version : " << ver_hex << LOG_END;
                            ms->storeMsVersion(ver_hex);
                            break;
                        }
                    if (msver_cb_) {
                        msver_cb_(msid, ver_hex);
                    }
                    }
                }
            );
        }else{            
            aero_command = new aero3::Aero3Command();
            aero_driver->setCommand(aero_command);
            //バージョン取得　暫定処理
            aero_driver->setMsVersionCallback(
                [this](int msid, const std::string& ver_hex){
                    int msdriver_id = 0;
                    MSDriver* target = nullptr;
                    for (auto* ms : ms_drivers) {
                        if (!ms) continue;
                        msdriver_id = ms->getMsId();
                        target = ms;
                        break;
                    }

                    bool should_process = false;
                    {
                        std::lock_guard<std::mutex> lk(msver_mtx_);
                        auto it = msver_cache_.find(msdriver_id);

                        if (it == msver_cache_.end()) {        
                            // 初回のみ保存
                            msver_cache_[msdriver_id] = ver_hex;
                            should_process = true;
                            target->storeMsVersion(ver_hex);
                        } else if (it->second != ver_hex) {
                            // 同じMSIDで version違い を検知
                            LOG_WARN_STREAM() << "MS firmware version changed. MSID=" << msdriver_id  << " old=" << it->second  << " new=" << ver_hex << LOG_END;
                            it->second = ver_hex;
                            should_process = false;
                        } else {
                          should_process = false;
                        }
                    }

                    if (!should_process) {
                        return;
                    }

                    LOG_INFO_STREAM() << "MSID : " << msdriver_id  << ", MS Firmware Version : " << ver_hex << LOG_END;

                    if (msver_cb_) {
                        msver_cb_(msdriver_id, ver_hex);
                    }
                }
            );
        }
    }

    //非RT
    std::string getProtocol() {
        if(aero_command){
            return aero_command->getProtocol();
        }
        return "";
    }

    //非RT
    bool hasMs(int msid){
        for(auto &ms_driver:ms_drivers){
            if(msid == ms_driver->getMsId()){
                return true;
            }
        }
        return false;
    }

    void addMsDriver(MSDriver* ms_driver){
        ms_drivers.push_back(ms_driver);
    }

    ~RobotDriverSingleUSB() {
        if(aero_driver){delete aero_driver; aero_driver = nullptr;}
        if(aero_command){delete aero_command; aero_command = nullptr;}
    }

    int getNumMs(){
        return ms_drivers.size();
    }

    int getMsId(int idx){
        return ms_drivers[idx]->getMsId();
    }

    int getNumJoints(){
        int num = 0;
        for(auto &ms_driver:ms_drivers){
            num += ms_driver->getNumJoints();
        }
        return num;
    }

    //非RT
    std::string getJointName(int idx) {
        int ofst = 0;
        size_t msidx = 0;
        for (msidx = 0; msidx < ms_drivers.size(); ++msidx) {
            auto ms_driver = ms_drivers[msidx];
            if ((idx - ofst) >= ms_driver->getNumJoints()) {
                ofst += ms_driver->getNumJoints();
            } else {
                break;
            }
        }
        if (msidx < ms_drivers.size()) {
            auto name = ms_drivers[msidx]->getJointName(idx - ofst);
            return name;
        }
        return "";
    }

    //非RT
    std::vector<std::string> getJointNamesMessageOrder(int msid){
        std::vector<std::string> ret;

        for(auto &ms_driver:ms_drivers){
            if(ms_driver->getMsId() == msid){
                ret = ms_driver->getJointNamesMessageOrder();
                break;
            }
        }

        return ret;
    }

    //非RT
    std::vector<std::string> getJointNames(){
        std::vector < std::string > ret;
        for(auto &ms_driver:ms_drivers){
            auto jnames = ms_driver->getJointNames();
            ret.insert(ret.end(), jnames.begin(), jnames.end());
        }
        return ret;
    }

    //非RT
    std::string getPortName(){
        return usb_port;
    }

    void read() {
        aero_driver->read();
    }


    void getPosition(int16_t* positions){
        int ofst = 0;
        for(auto &ms_driver:ms_drivers){
            ms_driver->getPosition(&positions[ofst],aero_driver);
            ofst += ms_driver->getNumJoints();
        }
    }
   
    void sendMsVersion() {        
        for (auto &ms_driver : ms_drivers) {
            if (!ms_driver) { continue; }          // 予防
            ms_driver->sendMsVersion(aero_driver);  // aero_driver のnullも確認推奨
        }
    }
   
    int getMsStatus(MSStatus *ms_status, int ofst) {
        for (size_t msidx = 0; msidx < ms_drivers.size(); ++msidx) {
            ms_drivers[msidx]->getMsStatus(ms_status[msidx + ofst].err_u16, aero_driver);
        }
        return ms_drivers.size();
    }

    void getUSBStatus(USBStatus& usb_status){
        usb_status.disconnect = !aero_driver->connected();
    }


    //速度は取得できない
//    void getActualVelocities(std::vector<int16_t> &velocities){
//        for(auto &ms_driver:ms_drivers){
//            ms_driver->getActualVelocities(velocities,aero_driver);
//        }
//    }

    void sendVelocity(const int16_t* vel) {
        int ofst = 0;
        for(auto &ms_driver:ms_drivers){
            ms_driver->sendVelocity(&vel[ofst],aero_driver);
            ofst += ms_driver->getNumJoints();
        }
    }

    //ROS形式の並び順でのストローク
    void sendPosition(const int16_t* pos, const double &tgt_time_sec) {
        int ofst = 0;
        for(auto &ms_driver:ms_drivers){
            ms_driver->sendPosition(&pos[ofst],tgt_time_sec,aero_driver);
            ofst += ms_driver->getNumJoints();
        }
    }

    void sendIdleCommand(){
        for(auto &ms_driver:ms_drivers){
            ms_driver->sendIdleCommand(aero_driver);
        }
    }


    void getOtherCommands(int msidx, BuffList &cmds) {
        ms_drivers[msidx]->getOtherCommands(cmds, aero_driver);
    }

    void sendOtherCommands(int msidx, BuffList &cmds) {
        ms_drivers[msidx]->sendOtherCommands(cmds, aero_driver);
    }

    std::optional<std::string> getMsVersion(int msid) const {
        for (auto* ms : ms_drivers) {
            if (ms && ms->getMsId()) {
                return ms->getMsVersion(); 
            }
        }
        return std::nullopt;
    }
    
    void setMsVersionCallback(std::function<void(int, const std::string&)> cb) {
        msver_cb_ = std::move(cb);
    }
    
private:
    std::string usb_port;
    int baudrate;
    AeroCommand* aero_command = nullptr;
    AeroDriver *aero_driver = nullptr;
    std::vector<MSDriver*> ms_drivers;//一つのmmsに複数のmsがつながっている可能性があるので。

    std::function<void(int, const std::string&)> msver_cb_;
    std::unordered_map<int, std::string> msver_cache_;
    std::mutex msver_mtx_;
};
