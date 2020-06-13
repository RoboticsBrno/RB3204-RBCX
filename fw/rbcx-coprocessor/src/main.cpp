#include <array>

#include "stm32f1xx_hal.h"

#include "utils/Debug.hpp"
#include "utils/TaskWrapper.hpp"

#include "Bsp.hpp"
#include "ButtonController.hpp"
#include "CdcUartTunnel.hpp"
#include "ControlLink.hpp"
#include "DebugLink.hpp"
#include "Dispatcher.hpp"
#include "StupidServoController.hpp"
#include "UltrasoundController.hpp"
#include "UsbCdcLink.h"

static TaskWrapper<2048> mainTask;

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
    ultrasoundInit();

    mainTask.start("main", 1, []() {
        DEBUG("STM32 Coprocessor initialized.\n");
        while (true) {
            cdcLinkPoll();
            tunnelPoll();
            dispatcherPoll();
            buttonControllerPoll();
        }
    });

    vTaskStartScheduler();
    abort();
}

// extern "C" void SysTick_Handler() {}
