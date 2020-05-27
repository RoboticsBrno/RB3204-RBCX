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
    tunnelUartInit();
    controlUartInit();
    pinsInit();
    cdcLinkInit();
    while (true) {
        cdcLinkPoll();
        tunnelPoll();
        EspMessage incoming = EspMessage_init_default;
        StmMessage outgoing = StmMessage_init_default;
        if (controlLinkRx(incoming) && controlLinkTxReady()) {
            controlLinkTx(outgoing);
        }
    }
}

extern "C" void SysTick_Handler() {
    HAL_IncTick();
}
