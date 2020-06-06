#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

#include "Bsp.hpp"
#include "CdcUartTunnel.hpp"
#include "ControlLink.hpp"
#include "Dispatcher.hpp"
#include "UsbCdcLink.h"
#include <array>

int main() {
    clocksInit();
    HAL_Init();
    tunnelUartInit();
    controlUartInit();
    pinsInit();
    cdcLinkInit();
    while (true) {
        cdcLinkPoll();
        tunnelPoll();
        dispatcherPoll();
    }
}

// extern "C" void SysTick_Handler() {}
