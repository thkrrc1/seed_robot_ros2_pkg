#pragma once

#include <seed_ros2_controller/command_interface/command_interface.hpp>
#include <seed_ros2_controller/command_interface/other_command/other_cmd_buff.hpp>

namespace seed_ros2_controller{
namespace command_interface{
class OtherCommandHandle{
public:
    OtherCommandHandle() = default;

    OtherCommandHandle(const int msid, const std::vector<std::string> &joint_names, const std::string &protocol, BuffList *buff_list_recv, BuffList *buff_list_send) :
            msid_(msid), joint_names_(joint_names), protocol_(protocol), buff_list_recv_(buff_list_recv), buff_list_send_(buff_list_send) {
    }


    bool send_cmd(BuffRaw *buf) {
        if (buff_list_send_->size >= buff_list_send_->capacity) {
            return false;
        }
        memcpy(&buff_list_send_->buf[buff_list_send_->size], buf, sizeof(BuffRaw));
        buff_list_send_->size++;
        return true;
    }

    void get_cmd(uint8_t header1, uint8_t header2, BuffList *list) {
        if (!buff_list_recv_) {
            return;
        }

        int idx2 = list->size;
        for (int idx = 0; idx < buff_list_recv_->size; ++idx) {
            if (buff_list_recv_->buf[idx].data[0] == header1 && buff_list_recv_->buf[idx].data[1] == header2) {
                memcpy(&list->buf[idx2], &buff_list_recv_->buf[idx], sizeof(BuffRaw));
                list->size = idx2 + 1;
                idx2++;
            }
        }

        return;
    }


    std::string get_name() const {
        return std::to_string(msid_);
    }

    std::vector<std::string> get_joint_names() const{
        return joint_names_;
    }

    std::string get_protocol() const{
        return protocol_;
    }
private:
    int msid_ = 0;
    std::vector<std::string> joint_names_;
    std::string protocol_ = "";

    BuffList *buff_list_recv_ = nullptr;
    BuffList *buff_list_send_ = nullptr;
};

class OtherCommandInterface : public CommandInterface<OtherCommandHandle>{
public:
    std::string get_interface_name() const override{
        return "other";
    }
};

}
}
