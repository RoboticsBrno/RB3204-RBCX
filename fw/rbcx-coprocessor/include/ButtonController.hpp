#pragma once

#include "Bsp.hpp"
#include "Dispatcher.hpp"
#include <stdint.h>

static uint32_t buttonState = 0;

inline void buttonControllerPoll() {
    uint32_t newButtonState = getButtons();
    if (buttonState != newButtonState) {
        auto status = CoprocStat();
        status.which_payload = CoprocStat_buttonsStat_tag;
        status.payload.buttonsStat.buttonsPressed
            = CoprocStat_ButtonsEnum(newButtonState);

        dispatcherEnqueueStatus(status);
        buttonState = newButtonState;
    }
}
