#pragma once

#include "ms_chain_usb.hpp"
#include "cosmo_joy.hpp"
#include "cosmo.hpp"
#include "key_input_mode.hpp"

class StatusInputMode : public KeyInputMode{

public:
    void addUsb(MSChainUSB *usbDriver){
        usb_infos.push_back(usbDriver);
    }

private:
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


    void execute(std::string input) override{
        std::vector<std::string> words = split(input,",");
        if(words.size() !=3){
            ROS_WARN("invalud request.");
            return;
        }

        int msid = std::stoi(words[0]);
        int bit = std::stoi(words[1]);
        int value = std::stoi(words[2]);

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
            tgt_driver->setStatusData(msid,bit,value);
        }
    }

    std::string getName() override{
        return "status mode";
    }

    void displayMenu() override{
        std::cout<<"\033[32m"<<"input ms status change request [msid,bit,up(1) or down(0)]"<< "\033[m"<<std::endl;
    }

private:
    std::vector<MSChainUSB*> usb_infos;
};
