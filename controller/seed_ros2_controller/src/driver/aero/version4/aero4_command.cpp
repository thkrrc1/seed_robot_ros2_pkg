#include "aero4_command.hpp"

#include "debug_tools.hpp"
#include "rt_logger/logger.hpp"

namespace aero4 {
///////////////////////////////
AeroCommand::AeroCommand() {
}

///////////////////////////////
AeroCommand::~AeroCommand() {
}

size_t AeroCommand::getExpectSize(const Header* header) {
    return 64;
}

PARSE_RESULT AeroCommand::parseData(uint8_t *recvd) {
    Header *header = reinterpret_cast<Header*>(recvd);

    PARSE_RESULT result = PARSE_RESULT::CMD_OTHER;
    if (header->data[0] == 0xDF && header->data[1] == 0xFD) {
        //aero
        RecvBuff *aero_cmd = reinterpret_cast<RecvBuff*>(recvd);
        if (!aero_cmd->isvalid()) {
            LOG_ERROR_STREAM() << "invalid checksum" << LOG_END;
            result = PARSE_RESULT::CMD_INVALID;
            return result;
        }

        if (aero_cmd->msid < 0 || aero_cmd->msid >= ms_num_capacity) {
            result = PARSE_RESULT::CMD_INVALID;
            return result;
        }
        MSState *tgt_state = &cur_state[aero_cmd->msid];

        if (aero_cmd->cmd == 0x14 || aero_cmd->cmd == 0x15 || aero_cmd->cmd == 0x41) {
            //制御命令応答(位置)
            int jidx = 0;
            for (int idx = 0; idx + 1 < aero_cmd->datalen() - 2; idx += 2) {
                aero_cmd->get(idx, tgt_state->mcval[jidx++].pos);
            }
            aero_cmd->get(aero_cmd->datalen() - 2, tgt_state->status);
            result = PARSE_RESULT::CMD_PARSED;
        } else if (aero_cmd->cmd == 0x42) {
            //電流指令値取得返信
            int jidx = 0;
            for (int idx = 0; idx + 1 < aero_cmd->datalen() - 2; idx += 2) {
                aero_cmd->get(idx, tgt_state->mcval[jidx++].current);
            }
            result = PARSE_RESULT::CMD_PARSED;
        } else if (aero_cmd->cmd == 0x43) {
            //温度・電圧取得返信
            int jidx = 0;
            for (int idx = 0; idx + 1 < aero_cmd->datalen() - 2; idx += 2) {
                aero_cmd->get(idx, tgt_state->mcval[jidx].voltage);
                aero_cmd->get(idx + 1, tgt_state->mcval[jidx].temp);
                ++jidx;
            }
            result = PARSE_RESULT::CMD_PARSED;
        }
    }

    return result;
}

void AeroCommand::sendPGET(SerialCommunication &serial_com, int msid) {
    buff.init();
    buff.header.data[0] = 0xFD;
    buff.header.data[1] = 0xDF;
    buff.cmd = 0x14;
    buff.msid = msid;

    int next_idx = 0;
    buff.setData((uint16_t)0x7FFF, 29, next_idx);

    buff.addChecksum();

    serial_com.write(buff.serialize(), buff.getTotalLen());
}

void AeroCommand::sendMOVE(SerialCommunication &serial_com, int msid, const double &tgt_time_sec, int16_t *data) {
    buff.init();
    buff.header.data[0] = 0xFD;
    buff.header.data[1] = 0xDF;
    buff.cmd = 0x14;
    buff.msid = msid;

    int next_idx = 0;
    uint16_t tgt_time = static_cast<uint16_t>(tgt_time_sec * 100);
    next_idx = buff.setData(data, 28, next_idx);
    next_idx = buff.setData(tgt_time, next_idx);

    buff.addChecksum();

    serial_com.write(buff.serialize(), buff.getTotalLen());
}

void AeroCommand::sendTURN(SerialCommunication &serial_com, int msid, int16_t *data) {
    buff.init();
    buff.header.data[0] = 0xFD;
    buff.header.data[1] = 0xDF;
    buff.cmd = 0x15;
    buff.msid = msid;

    int next_idx = 0;
    next_idx = buff.setData(data, 28, next_idx);
    next_idx = buff.setData(uint16_t(0x7FFF), next_idx);
    buff.addChecksum();

    serial_com.write(buff.serialize(), buff.getTotalLen());
}

int16_t AeroCommand::getpos(int msid, int joint) {
    return cur_state[msid].mcval[joint].pos;
}

uint16_t AeroCommand::getstatus(int msid) {
    return cur_state[msid].status;
}
}
