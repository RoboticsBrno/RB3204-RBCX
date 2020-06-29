#include "Dispatcher.hpp"
#include "FreeRTOS.h"
#include "utils/Debug.hpp"

#include "Bsp.hpp"
#include "BuzzerController.hpp"
#include "ControlLink.hpp"
#include "Esp32Manager.hpp"
#include "MotorController.hpp"
#include "StupidServoController.hpp"
#include "UltrasoundController.hpp"
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
    auto ok = xQueueSendToBack(statusQueue, &status, 0) == pdTRUE;
    if (!ok) {
        DEBUG("Status queue overflow\n");
    }
    return ok;
}

void dispatcherPoll() {
    if (controlLinkRx(request)) {
        status = CoprocStat_init_default;
        sEsp32Manager.resetWatchdog();
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
        case CoprocReq_setStupidServo_tag:
            stupidServoDispatch(request.payload.setStupidServo);
            break;
        case CoprocReq_ultrasoundReq_tag:
            ultrasoundDispatch(request.payload.ultrasoundReq);
            break;
        case CoprocReq_motorReq_tag:
            motorDispatch(request.payload.motorReq);
            break;
        case CoprocReq_buzzerReq_tag:
            buzzerSetState(request.payload.buzzerReq.on);
            break;
        }
    }

    if (xQueueReceive(statusQueue, &status, 0) == pdTRUE) {
        controlLinkTx(status);
    }
}
