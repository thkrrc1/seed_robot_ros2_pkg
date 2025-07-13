#pragma once

#include "cmd_data.hpp"

void dump(const char* title,uint16_t address,uint8_t len,uint8_t datalen,uint8_t* data,int ofst = 0);
void dump(const char* title,uint8_t datalen,uint8_t* data,int ofst = 0);
