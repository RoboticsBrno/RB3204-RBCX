#include "Bsp.hpp"
#include "BuzzerController.hpp"
#include "ControlLink.hpp"
#include "Dispatcher.hpp"
#include "Esp32Manager.hpp"
#include "MotorController.hpp"
#include "StupidServoController.hpp"
#include "UltrasoundController.hpp"
#include "utils/Debug.hpp"
#include "utils/TaskWrapper.hpp"

static TaskWrapper<1024> softResetTask;

#if RBCX_HW_VER == 0x0100
extern "C" void EXTI1_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1); }
extern "C" void EXTI3_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3); }
#elif RBCX_HW_VER == 0x0101
extern "C" void EXTI4_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4); }
#endif

extern "C" void EXTI9_5_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t pin) {
    if (pin == espEnPin.second) {
        sEsp32Manager.onEnRisingInIrq();
        return;
    }

    for (const auto& p : utsEchoPin) {
        if (pin == p.second) {
            ultrasoundOnEchoEdge();
            return;
        }
    }
}

void softResetInit() {
    softResetTask.start("softrst", configMAX_PRIORITIES - 1, []() {
        while (true) {
            if (xTaskNotifyWait(0, 0, nullptr, portMAX_DELAY) != pdTRUE)
                continue;

            //DEBUG("Soft resetting peripherials to default state.\n");

            setLeds(0);
            buzzerSetState(false);
            dispatcherReset();
            controlLinkReset();
            stupidServoReset();
            ultrasoundReset();
            motorReset();
        }
    });
}

void softReset() {
    if (isInInterrupt()) {
        BaseType_t woken = pdFALSE;
        xTaskNotifyFromISR(softResetTask.handle(), 0, eNoAction, &woken);
        portYIELD_FROM_ISR(woken);
    } else {
        xTaskNotify(softResetTask.handle(), 0, eNoAction);
    }
}
