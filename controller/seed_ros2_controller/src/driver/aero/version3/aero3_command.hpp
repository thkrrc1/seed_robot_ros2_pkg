#pragma once

#include "aero_driver.hpp"
#include <seed_ros2_controller/aero/version3/send_buff_aero3.hpp>
#include <seed_ros2_controller/aero/version3/recv_buff_aero3.hpp>

namespace aero3 {
class Aero3Command: public ::AeroCommand {
public:
    Aero3Command();
    ~Aero3Command();

    size_t getExpectSize(const Header* header) override;
    PARSE_RESULT parseData(uint8_t *recvd) override;

    void sendPGET(SerialCommunication &serial_com, int msid) override;
    void sendMOVE(SerialCommunication &serial_com, int msid, const double &tgt_time_sec, int16_t *data) override;
    void sendTURN(SerialCommunication &serial_com, int msid, int16_t *data) override;

    int16_t getpos(int msid, int joint) override;
    uint16_t getstatus(int msid) override;

    std::string getProtocol() override{
        return "aero3";
    }
private:
    static constexpr int ms_num_capacity = 100; //!< 通信可能なmsの数の最大値
    MSState cur_state[ms_num_capacity]; //msごとに用意

    SendBuff buff;
};
}
