#pragma once

#include "ms_chain_usb.hpp"
#include "cosmo_joy.hpp"
#include "cosmo.hpp"
#include "key_input_mode.hpp"
#include "log_macro.hpp"

class CosmoInputMode : public KeyInputMode{

public:

    void addUsb(MSChainUSB *usbDriver){
        usb_infos.push_back(usbDriver);
    }

private:
    void execute(std::string input) override{
        std::vector<std::string> words = split(input,",");
        if(words.size() !=2){
            ROS_WARN("invalud request.");
            return;
        }

        int msid = std::stoi(words[0]);
        char cosmo_cmd[100] = { 0 };
        memcpy(cosmo_cmd,words[1].c_str(),words[1].length());

        send(msid,cosmo_cmd);
    }

    std::vector<std::string> split(std::string str,std::string delim){
        std::vector<std::string> result;
        size_t pos = 0;
        while ((pos = str.find(delim)) != std::string::npos) {
            result.push_back(str.substr(0, pos));
            str.erase(0, pos + delim.length());
        }
        result.push_back(str);

        return result;
    }

    void send(int msid,std::string cosmo_cmd){
        ROS_INFO_STREAM("cosmo start -> msid: "<<msid<<" cmd: "<<cosmo_cmd);
        MSChainUSB* tgt_driver = nullptr;
        for(auto &usb_info:usb_infos){
            for(int info_msid:usb_info->getMsIds()){
                if(msid == info_msid){
                    tgt_driver = usb_info;
                    break;
                }
            }

            if(tgt_driver){
                break;
            }
        }
        if (tgt_driver) {
            CosmoSendRaw data;
            data.setHeader(msid);
            data.setData(cosmo_cmd);
            data.addChecksum();
            tgt_driver->sendOtherCmd(&data);
        }
    }

    std::string getName() override{
        return "cosmo mode";
    }

    void displayMenu() override{
        std::cout<<"\033[32m"<<"input cosmo request [msid,command]"<< "\033[m"<<std::endl;
    }

private:
    std::vector<MSChainUSB*> usb_infos;

};
