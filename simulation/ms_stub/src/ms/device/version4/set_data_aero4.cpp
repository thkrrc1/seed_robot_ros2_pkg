#include <cstring>
#include <iostream>

#include "set_data_aero4.hpp"
#include "tools.hpp"

namespace aero4{
void createMovePosResp(MSData *data, aero4::RecvBuff *recvd, aero4::SendBuff *sendd) {
    uint16_t tgt_time;
    recvd->get(sizeof(recvd->data) - 2, tgt_time);

    for (size_t jno = 0; jno < sizeof(recvd->data) / 2 - 1; jno++) {
        if (data->joint[jno].servo == 0x0000) {
            continue;            //servo off
        }

        int16_t tgt_pos = 0;
        recvd->get(jno * 2, tgt_pos);
        if (tgt_pos != 0x7FFF) {
            if (tgt_time == 0) {
                tgt_time = 2;
            }
            double tgt_time_sec = tgt_time / 100.;
            data->joint[jno].pos_tgt = tgt_pos;
            data->joint[jno].vel_cur = (data->joint[jno].pos_tgt - data->joint[jno].pos_cur) / tgt_time_sec;
        }
    }

    //送信処理
    sendd->init();
    sendd->header.data[0] = 0xDF;
    sendd->header.data[1] = 0xFD;
    sendd->cmd = recvd->cmd;
    sendd->msid = recvd->msid;
    for (size_t jno = 0; jno < sizeof(sendd->data) / 2 - 1; jno++) {
        sendd->setData(data->joint[jno].pos, jno * 2);
    }
    sendd->setData(data->status.can2, sizeof(sendd->data) - 2);
    sendd->setData(data->status.can1, sizeof(sendd->data) - 1);

    sendd->addChecksum();
}

void createMoveVelResp(MSData *data, aero4::RecvBuff *recvd, aero4::SendBuff *sendd) {

    uint16_t tgt_time;
    recvd->get(sizeof(recvd->data) - 2, tgt_time);

    //受信処理
    for (size_t jno = 0; jno < sizeof(recvd->data) / 2 - 1; jno++) {
        if (data->joint[jno].servo == 0x0000) {
            continue;            //servo off
        }

        int16_t tgt_vel = 0;
        recvd->get(jno * 2, tgt_vel);
        if (tgt_vel != 0x7FFF) {
            data->joint[jno].vel_cur = tgt_vel;
        }
    }

    //送信処理
    sendd->init();
    sendd->header.data[0] = 0xDF;
    sendd->header.data[1] = 0xFD;
    sendd->cmd = recvd->cmd;
    sendd->msid = recvd->msid;
    for (size_t jno = 0; jno < sizeof(sendd->data) / 2 - 1; jno++) {
        sendd->setData(data->joint[jno].pos, jno * 2);
    }
    sendd->setData(data->status.can2, sizeof(sendd->data) - 2);
    sendd->setData(data->status.can1, sizeof(sendd->data) - 1);

    sendd->addChecksum();
}

void execServoCmd(MSData *data, aero4::RecvBuff *recvd) {
    data->status.can1.motor_stat = false;
    data->status.can2.motor_stat = false;

    for (size_t jno = 0; jno < sizeof(recvd->data) / 2; jno++) {
        uint16_t servo = 0;
        recvd->get(jno * 2, servo);

        if (servo != 0x7FFF) {
            data->joint[jno].servo = servo;
        }
    }

    std::cout << "[ ";
    for (size_t jno = 0; jno <= sizeof(recvd->data); jno++) {
        if (data->joint[jno].servo == 0x0000) {
            std::cout << "OFF ";
            if (jno <= 15) { // 1つのCANに16軸
                data->status.can1.motor_stat = true;
            } else {
                data->status.can2.motor_stat = true;
            }
        }else{
            std::cout << "ON  ";
        }
    }
    std::cout << "]" << std::endl;

}

void createSetEEPROMResp(MSData *data, aero4::RecvBuff *recvd, aero4::SendBuff *sendd) {
    uint16_t address = (recvd->data[1] << 8) | recvd->data[0];

    sendd->init();
    sendd->header.data[0] = 0xCF;
    sendd->header.data[1] = 0xFC;
    sendd->cmd = recvd->cmd;
    sendd->msid = recvd->msid;
    memcpy(sendd->data, recvd->data, sizeof(recvd->data));
    sendd->addChecksum();

    dump("eeprom write", address, 64, recvd->datalen(), recvd->data);

    memcpy(&data->eeprom[address], &recvd->data[2], recvd->datalen() - 2);

}

void createGetEEPROMResp(MSData *data, aero4::RecvBuff *recvd, aero4::SendBuff *sendd) {
    uint16_t address = (recvd->data[1] << 8) | recvd->data[0];            //リトルエンディアンに変換

    //data
    sendd->init();

    //header
    sendd->header.data[0] = 0xCF;
    sendd->header.data[1] = 0xFC;
    sendd->cmd = recvd->cmd;

    //address
    sendd->data[0] = recvd->data[0];
    sendd->data[1] = recvd->data[1];

    memcpy(&sendd->data[2], &data->eeprom[address], sendd->datalen() - 2);
    sendd->addChecksum();
}

void createFirmwareVersion(aero4::RecvBuff *recvd, aero4::SendBuff *sendd) {
    sendd->init();
    sendd->header.data[0] = 0xDF;
    sendd->header.data[1] = 0xFD;
    sendd->cmd = recvd->cmd;
    sendd->msid = recvd->msid;
    sendd->setData((uint8_t) (recvd->msid * 0x01 + 0x01), 0);
    sendd->setData((uint8_t) (recvd->msid * 0x01 + 0x02), 1);
    sendd->setData((uint8_t) (recvd->msid * 0x01 + 0x03), 2);
    sendd->setData((uint8_t) (recvd->msid * 0x01 + 0x04), 3);
    sendd->setData((uint8_t) (recvd->msid * 0x01 + 0x05), 4);
    sendd->addChecksum();
}

void createGetposResp(MSData *data, aero4::RecvBuff *recvd, aero4::SendBuff *sendd) {
    sendd->init();
    sendd->header.data[0] = 0xDF;
    sendd->header.data[1] = 0xFD;
    sendd->cmd = recvd->cmd;
    sendd->msid = recvd->msid;
    for (size_t jno = 0; jno < sizeof(sendd->data) / 2 - 1; jno++) {
        sendd->setData(data->joint[jno].pos, jno * 2);
    }
    sendd->setData(data->status.can2, sizeof(sendd->data) - 2);
    sendd->setData(data->status.can1, sizeof(sendd->data) - 1);

    sendd->addChecksum();
}

void createGetvolResp(MSData *data, aero4::RecvBuff *recvd, aero4::SendBuff *sendd) {
    sendd->init();
    sendd->header.data[0] = 0xDF;
    sendd->header.data[1] = 0xFD;
    sendd->cmd = recvd->cmd;
    sendd->msid = recvd->msid;

    for (size_t jno = 0; jno < sizeof(sendd->data) / 2 - 1; jno++) {
        sendd->setData(data->joint[jno].tmp, jno * 2);
        sendd->setData(data->joint[jno].vol, jno * 2 + 1);
    }
    sendd->addChecksum();
}

void createGetStatusResp(MSData *data, aero4::RecvBuff *recvd, aero4::SendBuff *sendd) {
    sendd->header.data[0] = 0xDF;
    sendd->header.data[1] = 0xFD;
    sendd->cmd = recvd->cmd;
    sendd->msid = recvd->msid;
    sendd->setData(data->stat_detail.data,sizeof(data->stat_detail.data),0);
    sendd->addChecksum();
}


void createGetdioResp(MSData *data, aero4::RecvBuff *recvd, aero4::SendBuff *sendd) {
    sendd->header.data[0] = 0xDF;
    sendd->header.data[1] = 0xFD;
    sendd->cmd = recvd->cmd;
    sendd->msid = recvd->msid;

    for (size_t msno = 0; msno < sizeof(sendd->data) / 2; msno++) {
        sendd->setData(data->joint[msno].dio, msno * 2);
    }

    sendd->addChecksum();
}

void createUpperBodyMsData(uint8_t *data) {
    int ofst = 1;
    data[0 + ofst] = 1; // msid
    data[1 + ofst] = 0xFF; // smsid
    data[2 + ofst] = 1; // 機体設定(0:None 1:Noid 2:Lifter 3:Mover)
    data[3 + ofst] = 0; // Protocol(0:Aero3 1:Aero4)
    data[4 + ofst] = 1; // Use Cloud
}

void createUpperBodySendNoData(uint8_t *data) {
    int ofst = 1;
    //neck
    data[0 + ofst] = 10;
    data[1 + ofst] = 12;
    data[2 + ofst] = 11;

    //right
    data[3 + ofst] = 1;
    data[4 + ofst] = 2;
    data[5 + ofst] = 3;
    data[6 + ofst] = 4;
    data[7 + ofst] = 5;
    data[8 + ofst] = 6;
    data[9 + ofst] = 7;

    //waist
    data[10 + ofst] = 9;

    //reserved
    data[11 + ofst] = 0;
    data[12 + ofst] = 0;
    data[13 + ofst] = 0;
    data[14 + ofst] = 0;

    //waist
    data[15 + ofst] = 25;

    //reserved
    data[16 + ofst] = 0;
    data[17 + ofst] = 0;

    //left
    data[18 + ofst] = 16;
    data[19 + ofst] = 17;
    data[20 + ofst] = 18;
    data[21 + ofst] = 19;
    data[22 + ofst] = 20;
    data[23 + ofst] = 21;
    data[24 + ofst] = 22;

    //waist
    data[25 + ofst] = 24;

    //reserved
    data[26 + ofst] = 0;
    data[27 + ofst] = 0;
    data[28 + ofst] = 0;
    data[29 + ofst] = 0;
    data[30 + ofst] = 0;
}
}
