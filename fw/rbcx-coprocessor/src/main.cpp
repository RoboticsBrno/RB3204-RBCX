#include "stm32f1xx_hal.h"

#include "Bsp.hpp"
#include "ButtonController.hpp"
#include "CdcUartTunnel.hpp"
#include "ControlLink.hpp"
#include "Dispatcher.hpp"
#include "StupidServoController.hpp"
#include "UltrasoundController.hpp"
#include "UsbCdcLink.h"
#include <array>

int main() {
    clocksInit();
    HAL_Init();
    dispatcherInit();
    tunnelUartInit();
    controlUartInit();
    pinsInit();
    cdcLinkInit();
    stupidServoInit();
    ultrasoundInit();
    while (true) {
        cdcLinkPoll();
        tunnelPoll();
        dispatcherPoll();
        buttonControllerPoll();
    }
}

// extern "C" void SysTick_Handler() {}
