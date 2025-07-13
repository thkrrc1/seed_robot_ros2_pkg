#pragma once

#include <seed_ros2_controller/aero/version3/recv_buff_aero3.hpp>
#include <seed_ros2_controller/aero/version3/send_buff_aero3.hpp>

#include "ms_stub.hpp"
#include "set_data_aero3.hpp"
#include "log_macro.hpp"

namespace aero3{
class MSStubSingleAero3: public MSStubSingle {
public:
    MSStubSingleAero3(int msid) : MSStubSingle(msid) {
        createUpperBodyMsData(&getMsData().eeprom[0x0000]);
        createUpperBodySendNoData(&getMsData().eeprom[0x0100]);
    }

private:
    size_t getExpectSize(const uint8_t* buff) const override{
        if (buff[0] == 0xfe && buff[1] == 0xef) {
            return 64;
        } else if (buff[0] == 0xfe && buff[1] == 0xef) {
            return 64;
        }else if(buff[0] == 0xfd && buff[1] == 0xdf) {
            return buff[2] + 4;
        }else if(buff[0] == 0xfc && buff[1] == 0xcf) {
            return buff[2] + 4;
        }
        return 68;
    }

    bool accept(Serial &serial, uint8_t *recvd_raw, size_t len) override {
        auto recv_buff = reinterpret_cast<aero3::RecvBuff*>(recvd_raw);

        if (recv_buff->header.data[0] == 0xFD && recv_buff->header.data[1] == 0xDF) {
            execAeroCommand(serial, recv_buff);
        } else if (recv_buff->header.data[0] == 0xFC && recv_buff->header.data[1] == 0xCF) {
            execConfigCommand(serial, recv_buff);
        }
        return false; //MSIDの設定がないので、全MSに拾わせる
    }

    void execAeroCommand(Serial &serial, aero3::RecvBuff *recvd) {
        aero3::SendBuff sendd;
        if (recvd->cmd == 0x01) {            //電流値
            ROS_INFO("[%s] id:[%02d] cmd:[set current]", serial.getport().c_str(), recvd->mcid);
            for (int idx = 0; idx < (recvd->len - 2) / 2; ++idx) {
                uint8_t current_max = recvd->data[idx * 2];
                uint8_t current_min = recvd->data[idx * 2 + 1];
                if (current_max != 0x7F || current_min != 0xFF) {
                    ROS_INFO("   no: %d param -> max:[%d] min:[%d]", idx, current_max, current_min);
                }
            }
        } else if (recvd->cmd == 0x14) {    //位置司令
            createMovePosResp(&getMsData(), recvd, &sendd);
            serial.write(&sendd, sendd.getTotalLen());
        } else if (recvd->cmd == 0x15) {            //速度司令
            createMoveVelResp(&getMsData(), recvd, &sendd);
            serial.write(&sendd, sendd.getTotalLen());
        } else if (recvd->cmd == 0x21) {            //サーボON
            execServoCmd(&getMsData(), recvd);
            ROS_INFO("[%s] servo", serial.getport().c_str());
        } else if (recvd->cmd == 0x22) {            //スクリプト実行
            ROS_INFO("[%s] id:[%02d] cmd:[run script]", serial.getport().c_str(), recvd->mcid);
            for (int idx = 0; idx < (recvd->len - 2) / 2; ++idx) {
                uint8_t arg = recvd->data[idx * 2];
                uint8_t script_no = recvd->data[idx * 2 + 1];
                if (arg != 0x7F || script_no != 0xFF) {
                    ROS_INFO("   no:[%d] script:[%d] arg:[%d]", idx, script_no, arg);
                    //実行中は、自分のスクリプト番号が帰ってくる
                    getMsData().joint[idx].dio = script_no;
                }
            }
        } else if (recvd->cmd == 0x41) {    //位置取得
            createGetposResp(&getMsData(), recvd, &sendd);
            serial.write(&sendd, sendd.getTotalLen());
        } else if (recvd->cmd == 0x43) {            //電圧取得
            createGetvolResp(&getMsData(), recvd, &sendd);
            serial.write(&sendd, sendd.getTotalLen());
        } else if (recvd->cmd == 0x45) {   //DIO取得
            createGetdioResp(&getMsData(), recvd, &sendd);
            serial.write(&sendd, sendd.getTotalLen());
            for (int idx = 0; idx < AXIS_MAX; ++idx) {
                getMsData().joint[idx].dio = 0x00;   //DIOをリセットしておく
            }
        } else if (recvd->cmd == 0x51) {    //ファームウェアバージョン取得
            createFirmwareVersion(recvd, &sendd);
            serial.write(&sendd, sendd.getTotalLen());
        } else if (recvd->cmd == 0x61) {            //ステータス取得
            createGetStatusResp(getMsId(), &getMsData(), recvd, &sendd);
            serial.write(&sendd, sendd.getTotalLen());
        } else if (recvd->cmd == 0xa0 || recvd->cmd == 0xa1) {            //cosmo
            ROS_INFO("[%s] cosmo recv cmd:0x%02x data: %s", serial.getport().c_str(), recvd->cmd, recvd->data);
        }
    }

    void execConfigCommand(Serial &serial, aero3::RecvBuff *recvd) {
        aero3::SendBuff sendd;

        if (recvd->cmd == 0x00) {            //EPRAM読み込み
            createGetEEPROMResp(&getMsData(), recvd, &sendd);
            serial.write(&sendd, sendd.getTotalLen());
        } else if (recvd->cmd == 0xFF) {            //EPRAM書き込み
            createSetEEPROMResp(&getMsData(), recvd, &sendd);
            serial.write(&sendd, sendd.getTotalLen());
        } else if (recvd->cmd == 0x7F) {
            ROS_INFO("MS RESET");
        }
    }

};
}
