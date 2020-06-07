#include "Dispatcher.hpp"
#include "FreeRTOS.h"

#include "Bsp.hpp"
#include "ControlLink.hpp"
#include "queue.h"
#include "rbcx.pb.h"

static CoprocReq request;
static CoprocStat status;

static QueueHandle_t statusQueue;

void dispatcherInit() { statusQueue = xQueueCreate(64, CoprocStat_size); }

bool dispatcherEnqueueStatus(const CoprocStat& status) {
    return xQueueSendToBack(statusQueue, &status, 0) == pdTRUE;
}

void dispatcherPoll() {
    if (controlLinkRx(request)) {
        status = CoprocStat_init_default;
        switch (request.which_payload) {
        case CoprocReq_setLeds_tag:
            setLeds(request.payload.setLeds.ledsOn);
            status.which_payload = CoprocStat_ledsStat_tag;
            controlLinkTx(status);
            break;
        case CoprocReq_getButtons_tag:
            status.payload.buttonsStat.buttonsPressed
                = CoprocStat_ButtonsEnum(getButtons());
            status.which_payload = CoprocStat_buttonsStat_tag;
            controlLinkTx(status);
            break;
        }
    }
    if (xQueueReceive(statusQueue, &status, 0) == pdTRUE) {
        controlLinkTx(status);
    }
}
