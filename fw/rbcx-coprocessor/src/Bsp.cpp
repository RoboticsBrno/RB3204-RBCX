#include "Bsp.hpp"
#include "Esp32Manager.hpp"
#include "UltrasoundController.hpp"

extern "C" void EXTI1_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1); }

extern "C" void EXTI3_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3); }

extern "C" void EXTI9_5_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t pin) {
    if (pin == espEnPin.second) {
        sEsp32Manager.onEnRising();
    }
    if (pin & utsEchoPins.second) {
        onUltrasoundEchoEdge();
    }
}
