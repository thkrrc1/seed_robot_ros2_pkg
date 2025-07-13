#pragma once

#include "ms_data.hpp"

#include <seed_ros2_controller/aero/version4/recv_buff_aero4.hpp>
#include <seed_ros2_controller/aero/version4/send_buff_aero4.hpp>

namespace aero4 {
void createFirmwareVersion(aero4::RecvBuff *recvd, aero4::SendBuff *sendd);

void createGetposResp(MSData *data, aero4::RecvBuff *recvd, aero4::SendBuff *sendd);

void createGetvolResp(MSData *data, aero4::RecvBuff *recvd, aero4::SendBuff *sendd);

void createGetStatusResp(MSData *data, aero4::RecvBuff *recvd, aero4::SendBuff *sendd);

void createGetdioResp(MSData *data, aero4::RecvBuff *recvd, aero4::SendBuff *sendd);

void createGetEEPROMResp(MSData *data, aero4::RecvBuff *recvd, aero4::SendBuff *sendd);

void createSetEEPROMResp(MSData *data, aero4::RecvBuff *recvd, aero4::SendBuff *sendd);

void createMovePosResp(MSData *data, aero4::RecvBuff *recvd, aero4::SendBuff *sendd);

void createMoveVelResp(MSData *data, aero4::RecvBuff *recvd, aero4::SendBuff *sendd);

void execServoCmd(MSData *data, aero4::RecvBuff *recvd);

void createUpperBodyMsData(uint8_t *data);

void createUpperBodySendNoData(uint8_t *data);

}
