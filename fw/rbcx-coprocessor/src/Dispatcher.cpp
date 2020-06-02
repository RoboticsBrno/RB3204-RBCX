#include "Dispatcher.hpp"
#include "Bsp.hpp"
#include "ControlLink.hpp"
#include "rbcx.pb.h"

static CoprocReq request;
static CoprocStat status;

void dispatcherPoll() {
    if (controlLinkRx(request)) {
        bool ready = controlLinkTxReady();
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
}
