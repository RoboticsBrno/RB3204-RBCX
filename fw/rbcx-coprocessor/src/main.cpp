#include <array>

#include "stm32f1xx_hal.h"

#include "utils/Debug.hpp"
#include "utils/TaskWrapper.hpp"

#include "ButtonController.hpp"
#include "MotorController.hpp"
#include "StupidServoController.hpp"
#include "UltrasoundController.hpp"

#include "Bsp.hpp"
#include "CdcUartTunnel.hpp"
#include "ControlLink.hpp"
#include "DebugLink.hpp"
#include "Dispatcher.hpp"
#include "Esp32Manager.hpp"
#include "Power.hpp"
#include "RtcController.hpp"
#include "UsbCdcLink.h"

static TaskWrapper<3072> mainTask;

int main() {
#ifdef RBCX_VECT_TAB_OFFSET
    SCB->VTOR = FLASH_BASE | RBCX_VECT_TAB_OFFSET;
#endif

    // Allow POWER and LEDs drive in powerEarlyInit
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    pinInit(powerPin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    pinInit(ledPins, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);

    powerEarlyInit();
    clocksInit();
    HAL_Init();

    pinsInit();

    mainTask.start("main", 1, []() {
        debugUartInit();
        softResetInit();
        powerInit();
        rtcInit();
        dispatcherInit();
        tunnelUartInit();
        controlUartInit();
        cdcLinkInit();
        stupidServoInit();
        ultrasoundInit();
        sEsp32Manager.init();
        motorInit();

        DEBUG("STM32 Coprocessor initialized, v%06x " RBCX_VER_REVISION
                  RBCX_VER_DIRTY_STR "\n",
            RBCX_VER_NUMBER);
        while (true) {
            debugLinkPoll();
            powerPoll();
            dispatcherPoll();
            tunnelPoll();
            buttonControllerPoll();
            sEsp32Manager.poll();
        }
    });

    vTaskStartScheduler();
    abort();
}
