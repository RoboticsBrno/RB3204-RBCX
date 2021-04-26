#include <array>

#include "stm32f1xx_hal.h"

#include "utils/Debug.hpp"
#include "utils/TaskWrapper.hpp"

#include "ButtonController.hpp"
#include "I2cController.hpp"
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
#include "UsbCdcLink.h"

static TaskWrapper<3072> mainTask;
// static uint32_t _sysTickHigh = 0;

// #define SYSTICK_LENGTH 24
// uint32_t _sysTickToUpConting(uint32_t v) {
//     return (SysTick->LOAD & SysTick_LOAD_RELOAD_Msk)
//         - (SysTick->VAL & SysTick_VAL_CURRENT_Msk);
// }
// uint64_t _sysTickCombineHLcounters(uint32_t h, uint32_t l) {
//     return (uint64_t(h) << SYSTICK_LENGTH) | l;
// }

// uint64_t getSysTickTime() {
//     uint32_t th1 = _sysTickHigh;
//     uint32_t tl1 = SysTick->VAL;
//     uint32_t th2 = _sysTickHigh;
//     uint32_t tl2 = SysTick->VAL;
//     tl1 = _sysTickToUpConting(tl1);
//     tl2 = _sysTickToUpConting(tl2);
//     if (tl2 < tl1)
//         return _sysTickCombineHLcounters(th1, tl1);
//     return _sysTickCombineHLcounters(th2, tl2);
// }
// void systick_delay(uint64_t t) {
//     t += getSysTickTime();
//     while (t < getSysTickTime()) {
//     }
// }

int main() {
    clocksInit();
    HAL_Init();

#ifdef RBCX_VECT_TAB_OFFSET
    SCB->VTOR = FLASH_BASE | RBCX_VECT_TAB_OFFSET;
#endif

    pinsInit();

    mainTask.start("main", 1, []() {
        debugUartInit();
        softResetInit();
        powerInit();
        dispatcherInit();
        tunnelUartInit();
        controlUartInit();
        cdcLinkInit();
        i2cInit();
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

// extern "C" void SysTick_Handler() {
//     ++_sysTickHigh;
//     HAL_IncTick();
// }
