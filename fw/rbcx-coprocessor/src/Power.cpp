
#include "Power.hpp"
#include "Bsp.hpp"
#include "utils/Debug.hpp"

#include "rbcx.pb.h"

void powerShutDown() {
    DEBUG("Shutting down...\n");

    vTaskDelay(1);

    __disable_irq();

    pinWrite(powerPin, 0);

    // Blink red LED "pretty fast"
    uint32_t leds = 0;
    while (true) {
        leds ^= CoprocReq_LedsEnum_L3;
        setLeds(leds);

        for (volatile int i = 0; i < 200000; i++)
            ;
    }
}
