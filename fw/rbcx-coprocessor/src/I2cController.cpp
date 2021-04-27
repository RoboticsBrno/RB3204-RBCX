#include "I2cController.hpp"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_hal_i2c.h"

#include "utils/Debug.hpp"

static I2C_HandleTypeDef i2cHandle;
int i2cTimeout = 100;

void i2cInit() {
    i2cHandle.Instance = I2C1;
    i2cHandle.Init.ClockSpeed = 100000;
    i2cHandle.Init.DutyCycle = I2C_DUTYCYCLE_2; //I2C_DUTYCYCLE_16_9
    i2cHandle.Init.OwnAddress1 = 0;
    i2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2cHandle.Init.OwnAddress2 = 0;
    i2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2cHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&i2cHandle);
}

uint8_t i2cTransmit(uint16_t DevAddress, uint8_t* pData, uint8_t Size) {
    uint8_t transmit = HAL_I2C_Master_Transmit(
        &i2cHandle, DevAddress, pData, Size, i2cTimeout);
    return transmit;
}

uint8_t i2cReceive(uint16_t DevAddress, uint8_t* pData, uint8_t Size) {
    uint8_t receive = HAL_I2C_Master_Receive(
        &i2cHandle, DevAddress, pData, Size, i2cTimeout);
    return receive;
}

uint8_t i2cReady(uint16_t DevAddress, uint8_t Trials) {
    uint8_t ready
        = HAL_I2C_IsDeviceReady(&i2cHandle, DevAddress, Trials, i2cTimeout);
    return ready;
}
uint8_t i2cScanner() {
    uint8_t counter = 0;
    for (int range = 0; range <= 254; range++) {
        if (i2cReady(range, 2) == HAL_OK) {
            counter++;
            DEBUG("I2c Scan[%d] ready: %#04x (%d)\n", counter, range, range);
        }
    }
    return counter;
}