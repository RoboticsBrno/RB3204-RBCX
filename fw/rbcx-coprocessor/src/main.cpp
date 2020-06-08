#include <array>

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

#include "utils/Debug.hpp"
#include "utils/TaskWrapper.hpp"

#include "Bsp.hpp"
#include "ButtonController.hpp"
#include "CdcUartTunnel.hpp"
#include "ControlLink.hpp"
#include "DebugLink.hpp"
#include "Dispatcher.hpp"
#include "Esp32Manager.hpp"
#include "StupidServoController.hpp"
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

    mainTask.start("main", 1, []() {
        DEBUG("STM32 Coprocessor initialized.\n");
        while (true) {
            cdcLinkPoll();
            tunnelPoll();
            dispatcherPoll();
            buttonControllerPoll();
            sEsp32Manager.poll();
        }
    });

    vTaskStartScheduler();
    abort();
}

// extern "C" void SysTick_Handler() {}
