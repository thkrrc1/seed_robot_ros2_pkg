#include <cstring>
#include <iostream>

#include "set_data_aero3.hpp"
#include "tools.hpp"

namespace aero3{

void createMovePosResp(MSData *data, aero3::RecvBuff *recvd, aero3::SendBuff *sendd) {
    int mcno = 0;
    int data_len = 0;

    //受信処理
    data_len = recvd->len - 2;            //mcidとcmdを除く

    if (recvd->mcid == 0) {            //全軸送信
        uint16_t tgt_time = ((recvd->data[60] << 8) + recvd->data[61]);
        for (mcno = 0; mcno < (data_len - 2) / 2; mcno++) {
            if (data->joint[mcno].servo == 0x0000) {
                continue;            //servo off
            }

            if (!(recvd->data[mcno * 2] == 0x7f && recvd->data[mcno * 2 + 1] == 0xff)) {
                if (tgt_time == 0) {
                    tgt_time = 2;
                }
                double tgt_time_sec = tgt_time / 100.;
                int16_t tgt_pos = (recvd->data[mcno * 2] << 8) + recvd->data[mcno * 2 + 1];
                data->joint[mcno].pos_tgt = tgt_pos;
                data->joint[mcno].vel_cur = (data->joint[mcno].pos_tgt - data->joint[mcno].pos_cur) / tgt_time_sec;
            }
        }
    } else if (recvd->mcid - 1 < AXIS_MAX) {
        mcno = recvd->mcid - 1;
        if (data->joint[mcno].servo == 0x0000) {
            ;            //servo off
        } else if (!(recvd->data[mcno * 2] == 0x7f && recvd->data[mcno * 2 + 1] == 0xff)) {
            uint16_t tgt_time = 2;
            double tgt_time_sec = tgt_time / 100.;
            int16_t tgt_pos = (recvd->data[mcno * 2] << 8) + recvd->data[mcno * 2 + 1];
            data->joint[mcno].pos_tgt = tgt_pos;
            data->joint[mcno].vel_cur = (data->joint[mcno].pos_tgt - data->joint[mcno].pos_cur) / tgt_time_sec;
        }
    }

    //送信処理
    sendd->init();
    sendd->header.data[0] = 0xDF;
    sendd->header.data[1] = 0xFD;
    sendd->cmd = recvd->cmd;
    sendd->len = 64;
    sendd->mcid = recvd->mcid;
    if (recvd->mcid == 0) {
        for (mcno = 0; mcno < 30; mcno++) {
            sendd->data[mcno * 2 + 0] = ((uint8_t*) &data->joint[mcno].pos)[1];
            sendd->data[mcno * 2 + 1] = ((uint8_t*) &data->joint[mcno].pos)[0];
        }
        sendd->data[60] = *(uint8_t*) &data->status.can2;
        sendd->data[61] = *(uint8_t*) &data->status.can1;
    } else {
        mcno = recvd->mcid - 1;
        sendd->data[mcno * 2 + 0] = ((uint8_t*) &data->joint[mcno].pos)[1];
        sendd->data[mcno * 2 + 1] = ((uint8_t*) &data->joint[mcno].pos)[0];
    }

    sendd->addChecksum();
}

void createMoveVelResp(MSData *data, aero3::RecvBuff *recvd, aero3::SendBuff *sendd) {
    int mcno = 0;
    int data_len = 0;

    //受信処理

    data_len = recvd->len - 2;            //mcidとcmdを除く
    if (recvd->mcid == 0) {
        for (mcno = 0; mcno < (data_len - 2) / 2; mcno++) {
            if (data->joint[mcno].servo == 0x0000) {
                continue;            //servo off
            }

            if (!(recvd->data[mcno * 2] == 0x7f && recvd->data[mcno * 2 + 1] == 0xff)) {
                int16_t vel = (recvd->data[mcno * 2] << 8) + recvd->data[mcno * 2 + 1];
                data->joint[mcno].vel_cur = vel;
            }
        }
    } else if (recvd->mcid - 1 < AXIS_MAX) {
        mcno = recvd->mcid - 1;
        if (data->joint[mcno].servo == 0x0000) {
        } else if (!(recvd->data[mcno * 2] == 0x7f && recvd->data[mcno * 2 + 1] == 0xff)) {
            int16_t vel = (recvd->data[mcno * 2] << 8) + recvd->data[mcno * 2 + 1];
            data->joint[mcno].vel_cur = vel;
        }
    }

    //送信処理
    sendd->init();
    sendd->header.data[0] = 0xDF;
    sendd->header.data[1] = 0xFD;
    sendd->cmd = recvd->cmd;
    sendd->len = 64;
    sendd->mcid = recvd->mcid;
    if (recvd->mcid == 0) {
        for (mcno = 0; mcno < 30; mcno++) {
            sendd->data[mcno * 2 + 0] = ((uint8_t*) &data->joint[mcno].pos)[1];
            sendd->data[mcno * 2 + 1] = ((uint8_t*) &data->joint[mcno].pos)[0];
        }
        sendd->data[60] = *(uint8_t*) &data->status.can2;
        sendd->data[61] = *(uint8_t*) &data->status.can1;
    } else {
        mcno = recvd->mcid - 1;
        sendd->data[mcno * 2 + 0] = ((uint8_t*) &data->joint[mcno].pos)[1];
        sendd->data[mcno * 2 + 1] = ((uint8_t*) &data->joint[mcno].pos)[0];
    }

    sendd->addChecksum();
}

void execServoCmd(MSData *data, aero3::RecvBuff *recvd) {
    data->status.can1.motor_stat = false;
    data->status.can2.motor_stat = false;

    int data_len = recvd->len - 2;
    for (int mcno = 0; mcno < (data_len - 2) / 2; mcno++) {
        if (!(recvd->data[mcno * 2] == 0x7f && recvd->data[mcno * 2 + 1] == 0xff)) {
            uint16_t servo = (recvd->data[mcno * 2] << 8) + recvd->data[mcno * 2 + 1];
            data->joint[mcno].servo = servo;
        }
    }

    std::cout << "[ ";
    for (int mcno = 0; mcno <= 31; mcno++) {
        if (data->joint[mcno].servo == 0x0000) {
            std::cout << "OFF ";
            if (mcno <= 15) {
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

void createSetEEPROMResp(MSData *data, aero3::RecvBuff *recvd, aero3::SendBuff *sendd) {
    //recvd,senddともにチェックサムは不要な点に注意
    uint8_t datalen = 0;
    uint16_t address = (recvd->mcid << 8) | recvd->data[0];            //リトルエンディアンに変換

    sendd->init();
    sendd->header.data[0] = 0xCF;
    sendd->header.data[1] = 0xFC;

    sendd->len = recvd->len;

    sendd->cmd = recvd->cmd;
    sendd->mcid = recvd->mcid;
    memcpy(sendd->data, recvd->data, sizeof(recvd->data));

    datalen = recvd->len - 2;            //アドレスの2バイトを除く

    dump("eeprom write", address, recvd->len, datalen, &recvd->data[1]);

    memcpy(&data->eeprom[address], &recvd->data[1], datalen);

}

void createGetEEPROMResp(MSData *data, aero3::RecvBuff *recvd, aero3::SendBuff *sendd) {
    //recvdにはチェックサムがあるが,senddにはチェックサムは不要な点に注意
    uint16_t address = (recvd->mcid << 8) | recvd->data[0];            //リトルエンディアンに変換
    uint8_t data_len = recvd->data[1];            //要求データサイズ

    //data
    sendd->init();

    //header
    sendd->header.data[0] = 0xCF;
    sendd->header.data[1] = 0xFC;
    sendd->len = data_len + 2;            //アドレスの2バイトを足す
    sendd->cmd = recvd->cmd;

    //address
    sendd->mcid = recvd->mcid;
    sendd->data[0] = recvd->data[0];

    memcpy(&sendd->data[1], &data->eeprom[address], data_len);
    dump("eeprom read", address, sendd->len, data_len, &sendd->data[1]);
}

void createFirmwareVersion(aero3::RecvBuff *recvd, aero3::SendBuff *sendd) {
    sendd->init();
    sendd->header.data[0] = 0xDF;
    sendd->header.data[1] = 0xFD;
    sendd->len = 7;
    sendd->cmd = recvd->cmd;
    sendd->mcid = 0x00;
    sendd->data[0] = recvd->mcid * 0x01 + 0x01;
    sendd->data[1] = recvd->mcid * 0x01 + 0x02;
    sendd->data[2] = recvd->mcid * 0x01 + 0x03;
    sendd->data[3] = recvd->mcid * 0x01 + 0x04;
    sendd->data[4] = recvd->mcid * 0x01 + 0x05;
    sendd->addChecksum();
}

void createGetposResp(MSData *data, aero3::RecvBuff *recvd, aero3::SendBuff *sendd) {
    int mcno = 0;
    sendd->init();
    sendd->header.data[0] = 0xDF;
    sendd->header.data[1] = 0xFD;
    sendd->cmd = recvd->cmd;
    sendd->mcid = recvd->mcid;
    if (recvd->mcid == 0) {
        sendd->len = 64;
        for (mcno = 0; mcno < 30; mcno++) {
            sendd->data[mcno * 2 + 0] = ((uint8_t*) &data->joint[mcno].pos)[1];
            sendd->data[mcno * 2 + 1] = ((uint8_t*) &data->joint[mcno].pos)[0];
        }
        sendd->data[60] = *(uint8_t*) &data->status.can2;
        sendd->data[61] = *(uint8_t*) &data->status.can1;
    } else {
        sendd->len = 4;
        mcno = recvd->mcid - 1;
        sendd->data[mcno * 2 + 0] = ((uint8_t*) &data->joint[mcno].pos)[1];
        sendd->data[mcno * 2 + 1] = ((uint8_t*) &data->joint[mcno].pos)[0];
    }

    sendd->addChecksum();
}

void createGetvolResp(MSData *data, aero3::RecvBuff *recvd, aero3::SendBuff *sendd) {
    int data_len = 0;
    int mcno = 0;
    sendd->init();
    sendd->header.data[0] = 0xDF;
    sendd->header.data[1] = 0xFD;
    sendd->cmd = recvd->cmd;
    sendd->len = 64;
    sendd->mcid = 0x00;

    data_len = sendd->len - 2;
    for (mcno = 0; mcno < (data_len - 2) / 2; mcno++) {
        sendd->data[mcno * 2 + 0] = data->joint[mcno].tmp;
        sendd->data[mcno * 2 + 1] = data->joint[mcno].vol;
    }
    sendd->data[60] = 0xFF;
    sendd->data[61] = 0x00;

    sendd->addChecksum();
}

void createGetStatusResp(int msid, MSData *data, aero3::RecvBuff *recvd, aero3::SendBuff *sendd) {
    sendd->header.data[0] = 0xDF;
    sendd->header.data[1] = 0xFD;
    sendd->cmd = recvd->cmd;
    sendd->len = 64;
    sendd->mcid = 0;
    sendd->data[0] = msid;

    std::memcpy(&sendd->data[1], data->stat_detail.data, sizeof(data->stat_detail.data));

//    sendd->data[57] = 0x00;
//    sendd->data[58] = 0x00;

    sendd->data[59] = 0x00;
    sendd->data[60] = 0x00;
    sendd->data[61] = 0x00;
    sendd->addChecksum();
}

void createGetdioResp(MSData *data, aero3::RecvBuff *recvd, aero3::SendBuff *sendd) {
    sendd->header.data[0] = 0xDF;
    sendd->header.data[1] = 0xFD;
    sendd->cmd = recvd->cmd;
    sendd->len = 64;
    sendd->mcid = 0x00;

    for (int mcno = 0; mcno < 30; mcno++) {
        sendd->data[mcno * 2 + 0] = ((uint8_t*) &data->joint[mcno].dio)[1];
        sendd->data[mcno * 2 + 1] = ((uint8_t*) &data->joint[mcno].dio)[0];
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
