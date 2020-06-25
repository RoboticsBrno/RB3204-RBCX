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
#include "Esp32Manager.hpp"
#include "Power.hpp"
#include "StupidServoController.hpp"
#include "UltrasoundController.hpp"
#include "UsbCdcLink.h"

static TaskWrapper<2048> mainTask;

int main() {
    clocksInit();
    HAL_Init();
    pinsInit();

    mainTask.start("main", 1, []() {
        debugUartInit();
        powerInit();
        dispatcherInit();
        tunnelUartInit();
        controlUartInit();
        cdcLinkInit();
        stupidServoInit();
        ultrasoundInit();
        sEsp32Manager.init();

        DEBUG("STM32 Coprocessor initialized, v%06x " RBCX_VER_REVISION
                  RBCX_VER_DIRTY_STR "\n",
            RBCX_VER_NUMBER);
        while (true) {
            debugLinkPoll();
            powerPoll();
            dispatcherPoll();
            tunnelPoll();
            cdcLinkPoll();
            buttonControllerPoll();
            sEsp32Manager.poll();
        }
    });

    vTaskStartScheduler();
    abort();
}

// extern "C" void SysTick_Handler() {}
