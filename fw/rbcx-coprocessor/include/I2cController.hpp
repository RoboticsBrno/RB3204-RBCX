#pragma once

#include "rbcx.pb.h"

void i2cInit();

uint8_t i2cTransmit(uint16_t DevAddress, uint8_t* pData, uint8_t Size);
uint8_t i2cReceive(uint16_t DevAddress, uint8_t* pData, uint8_t Size);
uint8_t i2cReady(uint16_t DevAddress, uint8_t Trials);
uint8_t i2cScanner();

void i2cDispatch(const CoprocReq_I2cReq& req);
void inttolitend(uint32_t x, uint8_t* lit_int);