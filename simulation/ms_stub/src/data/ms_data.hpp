#pragma once

#include <stdint.h>
#include <mutex>
#include "status_data.hpp"

#define AXIS_MAX 31

struct SingleCANErrorInfo {
    bool connection  :1 = 0;
    bool calibration :1 = 0;
    bool motor_stat  :1 = 0;
    bool temperature :1 = 0;
    bool response    :1 = 0;
    bool step_out    :1 = 0;
    bool protective_stopped :1 = 0;
    bool power       :1 = 0;
};


struct MSErrorInfo {
    SingleCANErrorInfo can2;
    SingleCANErrorInfo can1;
};


typedef struct {
    int16_t pos_min = -32768; //最大値
    int16_t pos_max = 32767; //最大値 台車は±13602

    double pos_cur = 0;
    double pos_tgt = std::numeric_limits<double>::quiet_NaN();
    double vel_cur = 0;

    int16_t pos = 0;
    uint8_t vol = 0;
    uint8_t tmp = 0;

    uint16_t servo = 1;//デフォルトでサーボON

    uint16_t dio = 0;
} MSJointData;

struct MSData{
    MSErrorInfo status;
    StatusData stat_detail;
    MSJointData joint[AXIS_MAX];
    uint8_t eeprom[65535];
    std::mutex mtx;

    MSData(){
        memset(eeprom,0xFF,sizeof(eeprom));
    }
} ;

