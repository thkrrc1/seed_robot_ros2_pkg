#include "aero_driver.hpp"

#include <iostream>

#include "debug_tools.hpp"
#include "rt_logger/logger.hpp"

///////////////////////////////
AeroDriver::AeroDriver() {
}

///////////////////////////////
AeroDriver::~AeroDriver() {
}

bool AeroDriver::openPort(std::string _port, unsigned int _baud_rate) {
    return serial_com.open(_port, _baud_rate);
}

void AeroDriver::closePort() {
    serial_com.close();
}

bool AeroDriver::connected() {
    return serial_com.connected();
}

void AeroDriver::read() {
    int read_size = 1;
    int other_cmd_recvd_no = 0;
    other_cmd_recvd.size = 0;
    while (read_size != 0) {
        uint8_t recv_data[256] = {0};
#if 1 // 頭出し付きの受信
        //受信データの頭出し作業
        bool found = false;
        size_t rsize = 0;
        size_t rsize_tot = 0;
        read_size = 0;
        while (1) {
            recv_data[0] = recv_data[1];
            rsize = serial_com.read(&recv_data[1], 1);
            if (rsize == 0) {
                break;
            }

            if (recv_data[0] == 0x00 || recv_data[1] == 0x00) {
                continue;
            }

            //ヘッダ判定
            if ((((recv_data[0] & 0x0F) << 4) == (recv_data[1] & 0xF0)) && ((recv_data[0] & 0xF0) == ((recv_data[1] & 0x0F) << 4))) {
                found = true;
                break;
            }
        }

        //ヘッダらしきデータがない場合は受信不可
        if(!found){
            break;
        }

        rsize_tot += 2;
        Header *header = reinterpret_cast<Header*>(recv_data);
        size_t expect_size = getExpectSize(header);
        if(expect_size == 0){
            //データ長を受信
            rsize = serial_com.read(&recv_data[2], 1);
            if (rsize < 1) {
                break;
            }
            rsize_tot += rsize;
            expect_size = recv_data[2] + sizeof(Header) + 2;//チェックサム、ヘッダ、lenはlenのサイズに含まれないので、足しておく
        }

        //期待サイズになるまで受信データを展開
        uint8_t *head = &recv_data[rsize_tot];
        while(rsize_tot < expect_size){
            rsize = serial_com.read(head, expect_size - rsize_tot);
            if(rsize == 0){
                break;
            }
            rsize_tot += rsize;
            head += rsize;
        }

        //期待したデータ量が得られなかった場合は、受信失敗
        if (rsize_tot != expect_size) {
            LOG_ERROR_STREAM() << "invalid packet length -> recv :" << rsize_tot << " expect: " << expect_size << LOG_END;
            break;
        }

        read_size = rsize_tot;

#else //単純な受信
        read_size = serial_com.read(recv_data, sizeof(recv_data));
        if (read_size == 0) {
            break;
        }
#endif


        auto result = parseData(recv_data);
        if(result == PARSE_RESULT::CMD_OTHER){
            //その他コマンド
            memcpy(other_cmd_recvd.buf[other_cmd_recvd_no].data, recv_data, read_size);
            other_cmd_recvd.buf[other_cmd_recvd_no].size = read_size;
            other_cmd_recvd.size = other_cmd_recvd_no + 1;
            other_cmd_recvd_no++;
            //キャパシティを超えそうな場合は、次回にまわす。
            if (other_cmd_recvd.cmd_capacity() == other_cmd_recvd_no) {
                break;
            }
        }
    }
}

void AeroDriver::getOtherCommands(int msid, BuffList &cmds) {
    memcpy(&cmds, &this->other_cmd_recvd, sizeof(this->other_cmd_recvd));
}

void AeroDriver::sendOtherCommands(int msid, BuffList &cmds) {
    for (int idx = 0; idx < cmds.size; ++idx) {
        serial_com.write(cmds.buf[idx].data, cmds.buf[idx].size);
        cmds.buf[idx].size = 0;
    }
    cmds.size = 0;
}

