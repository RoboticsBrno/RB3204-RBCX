#include "I2cController.hpp"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_hal_i2c.h"

/* Default I2C address */
#define MPU6050_I2C_ADDR 0xD2

/* Who I am register value */
#define MPU6050_I_AM 0x69

/* MPU6050 registers */
#define MPU6050_AUX_VDDIO 0x01
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_MOTION_THRESH 0x1F
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_INT_STATUS 0x3A
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_TEMP_OUT_L 0x42
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48
#define MPU6050_MOT_DETECT_STATUS 0x61
#define MPU6050_SIGNAL_PATH_RESET 0x68
#define MPU6050_MOT_DETECT_CTRL 0x69
#define MPU6050_USER_CTRL 0x6A
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_2 0x6C
#define MPU6050_FIFO_COUNTH 0x72
#define MPU6050_FIFO_COUNTL 0x73
#define MPU6050_FIFO_R_W 0x74
#define MPU6050_WHO_AM_I 0x75

/* Gyro sensitivities in degrees/s */
#define MPU6050_GYRO_SENS_250 ((float)131)
#define MPU6050_GYRO_SENS_500 ((float)65.5)
#define MPU6050_GYRO_SENS_1000 ((float)32.8)
#define MPU6050_GYRO_SENS_2000 ((float)16.4)

/* Acce sensitivities in g/s */
#define MPU6050_ACCE_SENS_2 ((float)16384)
#define MPU6050_ACCE_SENS_4 ((float)8192)
#define MPU6050_ACCE_SENS_8 ((float)4096)
#define MPU6050_ACCE_SENS_16 ((float)2048)

static I2C_HandleTypeDef i2cHandle;

uint8_t initReturn = 50;
uint8_t testRead = 51;
uint8_t testWrite = 52;
uint8_t i2cHandleState = 53;

void i2cInit() {
    // __HAL_AFIO_REMAP_I2C1_ENABLE();
    // i2cHandle.Instance->CR1 |=  I2C_CR1_SWRST;
    // i2cHandle.Instance->CR1 &=  ~I2C_CR1_SWRST;

    i2cHandle.Instance = I2C1;
    i2cHandle.Init.ClockSpeed = 100000;
    i2cHandle.Init.DutyCycle = I2C_DUTYCYCLE_2; //I2C_DUTYCYCLE_16_9
    i2cHandle.Init.OwnAddress1 = 0;
    i2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2cHandle.Init.OwnAddress2 = 0;
    i2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2cHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    // HAL_I2C_Init(&i2cHandle);
    initReturn = HAL_I2C_Init(&i2cHandle);  


}

uint8_t i2cInitRet() { return initReturn; }
uint8_t i2cTestReadRet() { return testRead; }
uint8_t i2cTestWriteRet() { return HAL_GetTick(); }

uint8_t i2cInitRetState() {
    i2cHandleState = i2cHandle.State;
    return i2cHandleState;
}

uint8_t i2cTest() {
    // testRead = HAL_I2C_IsDeviceReady(&i2cHandle,MPU6050_I2C_ADDR,2,100);
    // testRead = HAL_I2C_IsDeviceReady(&i2cHandle,0x55,2,100);

    uint8_t txBuffer[1] = { MPU6050_WHO_AM_I };
    testWrite = HAL_I2C_Master_Transmit(
        &i2cHandle, MPU6050_I2C_ADDR >> 1, txBuffer, sizeof(txBuffer), 1000);
    uint8_t rxBuffer[1];
    testRead = HAL_I2C_Master_Receive(
        &i2cHandle, MPU6050_I2C_ADDR >> 1, rxBuffer, sizeof(rxBuffer), 1000);

    return testRead;
}

uint8_t i2cSetup() {
    uint8_t txBuffer[2] = { MPU6050_PWR_MGMT_1, 0x00 };
    testWrite = HAL_I2C_Master_Transmit(
        &i2cHandle, MPU6050_I2C_ADDR >> 1, txBuffer, sizeof(txBuffer), 1000);     
    return testWrite;
}


int16_t i2cTemp() {
    uint8_t txBuffer[2] = { MPU6050_TEMP_OUT_H, 1};
    testWrite = HAL_I2C_Master_Transmit(
        &i2cHandle, MPU6050_I2C_ADDR >> 1, txBuffer, sizeof(txBuffer), 1000);
    
    uint8_t rxBuffer[2];
    testRead = HAL_I2C_Master_Receive(
        &i2cHandle, MPU6050_I2C_ADDR >> 1, rxBuffer, 2, 1000);

    int16_t raw_temp = (int16_t) (rxBuffer[0] << 8 | rxBuffer[1]);
    int16_t temp = (float) ((int16_t) raw_temp / (float) 340.0 + (float) 36.53);
    return temp;
}

// int i2cWrite() {

//     uint8_t buffer[] = { 0x6B, 0 };
//     HAL_StatusTypeDef ret
//         = HAL_I2C_Master_Transmit(&i2cHandle, 0x68, buffer, 2, 1);
//     if (ret == HAL_OK) {
//         printf("I2C OK");
//     }
//     return 55;
// }

// int i2cRead() {

//     uint8_t buffer[] = { 0x3B };
//     return HAL_I2C_Master_Transmit(&i2cHandle, 0x68, buffer, 1, 1);
//     // if(ret == HAL_OK)
//     // {
//     // 	printf("I2C OK");
//     // }
//     // return int(ret);
// }
