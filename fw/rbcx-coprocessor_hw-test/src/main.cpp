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
    uint8_t leds = 0x01;
    const uint32_t ledPeriod = 500;
    uint32_t nextLedTime = ledPeriod;
    while (true) {
        if (HAL_GetTick() >= nextLedTime) {
            for (uint8_t i = 0; i != ledPin.size(); ++i)
                pinWrite(ledPin[i], leds & (1<<i));
            switch (leds) {
            case 0x01: leds = 0x03; break;
            case 0x30: leds = 0x01; break;
            default: leds <<= 1; break;
            }
            nextLedTime += ledPeriod;
        }
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
}
