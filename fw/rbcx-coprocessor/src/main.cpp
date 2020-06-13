#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

#include "Bsp.hpp"
#include "ButtonController.hpp"
#include "CdcUartTunnel.hpp"
#include "ControlLink.hpp"
#include "DebugLink.hpp"
#include "Dispatcher.hpp"
#include "StupidServoController.hpp"
#include "UsbCdcLink.h"
#include "utils/Debug.hpp"
#include <array>

int main() {
    clocksInit();
    HAL_Init();
    pinsInit();
    debugUartInit();
    dispatcherInit();
    tunnelUartInit();
    controlUartInit();
    cdcLinkInit();
    stupidServoInit();

    DEBUG("STM32 Coprocessor initialized.\n");
    while (true) {
        cdcLinkPoll();
        tunnelPoll();
        dispatcherPoll();
        buttonControllerPoll();
    }
}

// extern "C" void SysTick_Handler() {}
