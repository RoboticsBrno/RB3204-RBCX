#include "Dispatcher.hpp"
#include "FreeRTOS.h"

#include "Bsp.hpp"
#include "ControlLink.hpp"
#include "queue.h"
#include "rbcx.pb.h"

static CoprocReq request;
static CoprocStat status;

static StaticQueue_t _statusQueueStruct;
static uint8_t _statusQueueItems[64 * CoprocStat_size];
static QueueHandle_t statusQueue;

void dispatcherInit() {
    statusQueue
        = xQueueCreateStatic(sizeof(_statusQueueItems) / CoprocStat_size,
            CoprocStat_size, _statusQueueItems, &_statusQueueStruct);
}

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
