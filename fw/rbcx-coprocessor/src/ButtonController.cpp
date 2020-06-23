#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "Bsp.hpp"
#include "Dispatcher.hpp"
#include "Power.hpp"

static uint32_t buttonState = 0;
static uint32_t powerOffAt = 0;

void buttonControllerPoll() {
    if (powerOffAt != 0 && xTaskGetTickCount() >= powerOffAt) {
        powerShutDown();
        powerOffAt = 0;
    }

    const uint32_t newButtonState = getButtons();
    if (buttonState == newButtonState)
        return;

    auto status = CoprocStat();
    status.which_payload = CoprocStat_buttonsStat_tag;
    status.payload.buttonsStat.buttonsPressed
        = CoprocStat_ButtonsEnum(newButtonState);

    dispatcherEnqueueStatus(status);
    buttonState = newButtonState;

    if ((newButtonState & CoprocStat_ButtonsEnum_BOFF) != 0) {
        powerOffAt = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
    } else {
        powerOffAt = 0;
    }

    if ((newButtonState & CoprocStat_ButtonsEnum_BON) != 0) {
        pinWrite(powerPin, 1);
    }
}
