#pragma once

#include "ms_data.hpp"

#include <seed_ros2_controller/aero/version3/recv_buff_aero3.hpp>
#include <seed_ros2_controller/aero/version3/send_buff_aero3.hpp>


namespace aero3{
void createFirmwareVersion(aero3::RecvBuff *recvd, aero3::SendBuff *sendd);

void createGetposResp(MSData *data, aero3::RecvBuff *recvd, aero3::SendBuff *sendd);

void createGetvolResp(MSData *data, aero3::RecvBuff *recvd, aero3::SendBuff *sendd);

void createGetStatusResp(int msid, MSData *data, aero3::RecvBuff *recvd,aero3::SendBuff *sendd);

void createGetdioResp(MSData *data, aero3::RecvBuff *recvd, aero3::SendBuff *sendd);

void createGetEEPROMResp(MSData *data, aero3::RecvBuff *recvd, aero3::SendBuff *sendd);

void createSetEEPROMResp(MSData *data, aero3::RecvBuff *recvd, aero3::SendBuff *sendd);

void createMovePosResp(MSData *data, aero3::RecvBuff *recvd,aero3::SendBuff *sendd);

void createMoveVelResp(MSData *data, aero3::RecvBuff *recvd,aero3::SendBuff *sendd);

void execServoCmd(MSData *data, aero3::RecvBuff *recvd);

void createUpperBodyMsData(uint8_t *data);

void createUpperBodySendNoData(uint8_t *data);
}
