#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

#include "Bsp.hpp"
#include "CdcUartTunnel.hpp"
#include "UsbCdcLink.h"

int main() {
    systemClocksInit();
    peripheralClocksInit();
    HAL_Init();
    primaryUartInit();
    pinsInit();
    cdcLinkInit();
    while (true) {
        cdcLinkPoll();
        tunnelPoll();
    }
}

extern "C" void SysTick_Handler() {
    HAL_IncTick();
}
