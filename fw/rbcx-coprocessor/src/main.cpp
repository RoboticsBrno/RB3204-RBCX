#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

#include <array>
#include "Bsp.hpp"
#include "CdcUartTunnel.hpp"
#include "UsbCdcLink.h"
#include "ControlLink.hpp"

int main() {
    clocksInit();
    HAL_Init();
    primaryUartInit();
    secondaryUartInit();
    pinsInit();
    cdcLinkInit();
    while (true) {
        cdcLinkPoll();
        tunnelPoll();
        std::array<uint8_t, 255> loopback;
        auto len = controlLinkRxFrame(loopback.data(), loopback.size());
        if (len && controlLinkTxReady()) {
            controlLinkTxFrame(loopback.data(), len);
        }
    }
}

extern "C" void SysTick_Handler() {
    HAL_IncTick();
}