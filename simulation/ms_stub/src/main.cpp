#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

#include "ms_chain_usb.hpp"
#include "cosmo_joy.hpp"
#include "cosmo.hpp"
#include "cosmo_input_mode.hpp"
#include "status_input_mode.hpp"
#include "key_input_thread.hpp"

int main(int argc, char **argv){
    std::vector<std::string> non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("my_publisher");

    if(non_ros_args.size() != 2){
        RCLCPP_FATAL(rclcpp::get_logger("ms_stub"), "driver settings is not specified.");
        return -1;
    }

    CosmoInputMode cosmo_mode;
    StatusInputMode status_mode;
    MSChainUSB *joy_usb = nullptr;

    RCLCPP_INFO_STREAM(rclcpp::get_logger("ms_stub"), "config path : "<<non_ros_args[1]);
    std::string driver_settings_path(non_ros_args[1]);
    YAML::Node config = YAML::LoadFile(driver_settings_path);

    auto usb_settings = config["usb_settings"];
    auto ms_settings = config["ms_settings"];

    for (size_t usb_idx = 0; usb_idx < usb_settings.size(); ++usb_idx) {
        auto param = usb_settings[usb_idx];
        if (!param["port"] || !param["mslist"]) {
            continue;
        }

        auto usb_port = param["port"].as<std::string>();
        std::filesystem::path path = usb_port;
        std::filesystem::path abspath = std::filesystem::absolute(usb_port);
        std::filesystem::create_directories(path.parent_path());

        std::string protocol = "";
        if(param["protocol"]){
            protocol = param["protocol"].as<std::string>();
        }

        MSChainUSB *usbDriver = new MSChainUSB(usb_port);
        if(!joy_usb){joy_usb = usbDriver;}

        auto mslist = param["mslist"];
        for (size_t ms_idx = 0; ms_idx < mslist.size(); ++ms_idx) {
            auto key = mslist[ms_idx].as<std::string>();

            if (ms_settings[key]) {
                auto ms_setting = ms_settings[key];
                int msid = ms_setting["msid"].as<int>();
                usbDriver->addMs(msid,protocol);

                auto joint_settings = ms_setting["joints"];
                for(size_t js_idx = 0; js_idx < joint_settings.size();++js_idx){
                    auto joint_setting = joint_settings[js_idx];
                    if (joint_setting["aero_idx"] && joint_setting["enc_range"]) {
                        int jidx = joint_setting["aero_idx"].as<int>();
                        int enc_min = joint_setting["enc_range"][0].as<int>();
                        int enc_max = joint_setting["enc_range"][1].as<int>();
                        usbDriver->setJointEncRange(msid, jidx, enc_min, enc_max);
                    }
                }
            }
        }

        cosmo_mode.addUsb(usbDriver);
        status_mode.addUsb(usbDriver);

    }

    KeyInputThread keyInput({&cosmo_mode,&status_mode});
    CosmoJoy joy(joy_usb);

    keyInput.run();

    rclcpp::spin(node);
    return 0;
}
