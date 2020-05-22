#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

#include <array>
#include "Bsp.hpp"
#include "CdcUartTunnel.hpp"
#include "UsbCdcLink.h"
#include "ControlUart.hpp"

int main() {
    clocksInit();
    HAL_Init();
    tunelUartInit();
    controlUartInit();
    pinsInit();
    cdcLinkInit();
    while (true) {
        cdcLinkPoll();
        tunnelPoll();
        std::array<uint8_t, 255> loopback;
        auto len = controlUartRxFrame(loopback.data(), loopback.size());
        if (len && controlUartTxReady()) {
            controlUartTxFrame(loopback.data(), len);
        }
    }
}

extern "C" void SysTick_Handler() {
    HAL_IncTick();
    HAL_GPIO_TogglePin(led1Pin.first, led1Pin.second);
}
