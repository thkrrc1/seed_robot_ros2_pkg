#pragma once

#include <seed_ros2_controller/aero/version4/recv_buff_aero4.hpp>
#include <seed_ros2_controller/aero/version4/send_buff_aero4.hpp>

#include "ms_stub.hpp"
#include "set_data_aero4.hpp"
#include "log_macro.hpp"
namespace aero4{
class MSStubSingleAero4: public MSStubSingle {
public:
    MSStubSingleAero4(int msid) : MSStubSingle(msid){
    }

private:
    size_t getExpectSize(const uint8_t* buff) const override{
        return 64;
    }

    bool accept(Serial &serial, uint8_t *recvd_raw, size_t len) override {
        auto recv_buff = reinterpret_cast<aero4::RecvBuff*>(recvd_raw);

        if(recv_buff->msid != getMsId()){
            return false;
        }

        if (recv_buff->header.data[0] == 0xFD && recv_buff->header.data[1] == 0xDF) {
            return execAeroCommand(serial, recv_buff);
        } else if (recv_buff->header.data[0] == 0xFC && recv_buff->header.data[1] == 0xCF) {
            return execConfigCommand(serial, recv_buff);
        }
        return false;
    }

    bool execAeroCommand(Serial &serial, aero4::RecvBuff *recvd) {
        aero4::SendBuff sendd;
        if (recvd->cmd == 0x01) {//電流値
            for (size_t idx = 0; idx < sizeof(recvd->data)/2; ++idx) {
                uint8_t current_max = recvd->data[idx * 2];
                uint8_t current_min = recvd->data[idx * 2 + 1];
                if (current_max != 0x7F || current_min != 0xFF) {
                    ROS_INFO("   no: %lu param -> max:[%d] min:[%d]", idx, current_max, current_min);
                }
            }
        } else if (recvd->cmd == 0x14) { //位置司令
            createMovePosResp(&getMsData(), recvd, &sendd);
            serial.write(&sendd, sendd.getTotalLen());
        } else if (recvd->cmd == 0x15) { //速度司令
            createMoveVelResp(&getMsData(), recvd, &sendd);
            serial.write(&sendd, sendd.getTotalLen());
        } else if (recvd->cmd == 0x15) { //サーボON
            execServoCmd(&getMsData(), recvd);
            ROS_INFO("[%s] servo", serial.getport().c_str());
        } else if (recvd->cmd == 0x22) { //スクリプト実行
            ROS_INFO("[%s] msid:[%02d] cmd:[run script]", serial.getport().c_str(), recvd->msid);
            for (size_t idx = 0; idx < sizeof(recvd->data)/2; ++idx) {
                uint8_t arg       = recvd->data[idx * 2 + 1];
                uint8_t script_no = recvd->data[idx * 2];
                if (script_no != 0x00) {
                    ROS_INFO("   no:[%ld] script:[%d] arg:[%d]", idx, script_no, arg);
                    //実行中は、自分のスクリプト番号が帰ってくる
                    getMsData().joint[idx].dio = script_no;
                }
            }
        } else if (recvd->cmd == 0x41) {//位置取得
            createGetposResp(&getMsData(), recvd, &sendd);
            serial.write(&sendd, sendd.getTotalLen());
        } else if (recvd->cmd == 0x43) {//電圧取得
            createGetvolResp(&getMsData(), recvd, &sendd);
            serial.write(&sendd, sendd.getTotalLen());
        } else if (recvd->cmd == 0x45) {   //DIO取得
            createGetdioResp(&getMsData(), recvd, &sendd);
            serial.write(&sendd, sendd.getTotalLen());
            for (int idx = 0; idx < AXIS_MAX; ++idx) {
                getMsData().joint[idx].dio = 0x00;   //DIOをリセットしておく
            }
        } else if (recvd->cmd == 0x51) {//ファームウェアバージョン取得
            createFirmwareVersion(recvd, &sendd);
            serial.write(&sendd, sendd.getTotalLen());
        } else if (recvd->cmd == 0x61) {//ステータス取得
            createGetStatusResp(&getMsData(), recvd, &sendd);
            serial.write(&sendd, sendd.getTotalLen());
        } else if (recvd->cmd == 0xa0 || recvd->cmd == 0xa1) {//cosmo
            ROS_INFO("[%s] cosmo recv cmd:0x%02x data: %s", serial.getport().c_str(), recvd->cmd, recvd->data);
        }

        return true;
    }

    bool execConfigCommand(Serial &serial, aero4::RecvBuff *recvd) {
        aero4::SendBuff sendd;
        if (recvd->cmd == 0x00) {            //EPRAM読み込み
            createGetEEPROMResp(&getMsData(), recvd, &sendd);
            serial.write(&sendd, sendd.getTotalLen());
        } else if (recvd->cmd == 0xFF) {            //EPRAM書き込み
            createSetEEPROMResp(&getMsData(), recvd, &sendd);
            serial.write(&sendd, sendd.getTotalLen());
        } else if (recvd->cmd == 0x7F) {
            ROS_INFO("MS RESET");
        }

        return true;
    }

};
}
