#pragma once

#include "rbcx.pb.h"
// #include "I2Cdev.h"

// typedef struct {

//     int16_t Accel_X_RAW;
//     int16_t Accel_Y_RAW;
//     int16_t Accel_Z_RAW;
//     double Ax;
//     double Ay;
//     double Az;

//     int16_t Gyro_X_RAW;
//     int16_t Gyro_Y_RAW;
//     int16_t Gyro_Z_RAW;
//     double Gx;
//     double Gy;
//     double Gz;

//     float Temperature;

//     double KalmanAngleX;
//     double KalmanAngleY;
// } MPU6050_t;

// MPU6050_t DataStruct;

void i2cInit();

uint8_t i2cInitRet();
uint8_t i2cTestReadRet();
uint8_t i2cTestWriteRet();

uint8_t i2cTest();
uint8_t i2cSetup();
int16_t i2cTemp();

int i2cWrite();
int i2cRead();

// We ensure RTC is running (noop in case it is)
// LL_I2C_InitTypeDef init;
//     // .AsynchPrescaler = 0x7FFF, // 32kHz crystal -> 1s tick
// init.PeripheralMode = LL_I2C_MODE_I2C;
// init.ClockSpeed = LL_I2C_MAX_SPEED_STANDARD;
// init.DutyCycle = LL_I2C_DUTYCYCLE_2;
// init.OwnAddress1 = 0x68;
// init.TypeAcknowledge = LL_I2C_ACK;
// init.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;

// LL_I2C_Init(I2C1, &init);
// HAL_I2C_Init()
