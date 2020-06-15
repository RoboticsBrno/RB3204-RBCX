#include "UltrasoundController.hpp"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_tim.h"
#include <optional>

#include "Bsp.hpp"
#include "Dispatcher.hpp"

static const uint16_t pingTimeoutMicros = 30'000;
static CoprocStat status;
static std::optional<int> utsActiveIndex;
static uint16_t risingEdgeMicros;

void ultrasoundInit() {
    LL_TIM_InitTypeDef timInit;
    LL_TIM_StructInit(&timInit);
    auto apb1TimClk = 2 * HAL_RCC_GetPCLK1Freq();

    timInit.Autoreload = pingTimeoutMicros;
    timInit.Prescaler = apb1TimClk / 1'000'000; // 1 us ticks
    LL_TIM_Init(utsTimer, &timInit);
    LL_TIM_SetOnePulseMode(utsTimer, LL_TIM_ONEPULSEMODE_SINGLE);
    LL_TIM_SetUpdateSource(utsTimer, LL_TIM_UPDATESOURCE_COUNTER);
    LL_TIM_ClearFlag_UPDATE(utsTimer);

    LL_EXTI_DisableIT_0_31(utsEchoPins.second);
    HAL_NVIC_SetPriority(utsTimerIRQn, utsIRQPrio, 0);
    HAL_NVIC_EnableIRQ(utsTimerIRQn);
}

void ultrasoundDispatch(const CoprocReq_UltrasoundReq& request) {
    if (request.utsIndex > 3) {
        return;
    }

    switch (request.which_utsCmd) {
    case CoprocReq_UltrasoundReq_singlePing_tag:
        LL_TIM_DisableIT_UPDATE(utsTimer);
        LL_EXTI_DisableIT_0_31(utsEchoPins.second);
        utsActiveIndex = request.utsIndex;
        LL_TIM_GenerateEvent_UPDATE(utsTimer);
        LL_TIM_EnableCounter(utsTimer);

        pinWrite(utsTrigPin[request.utsIndex], true);
        while (LL_TIM_GetCounter(utsTimer) < 10
            && !LL_TIM_IsActiveFlag_UPDATE(utsTimer)) { // 10 us pulse
        }
        pinWrite(utsTrigPin[request.utsIndex], false);

        LL_TIM_EnableIT_UPDATE(utsTimer);
        LL_EXTI_EnableIT_0_31(utsEchoPin[request.utsIndex].second);
        break;
    }
}

static void enqueueStatus(int utsIndex, uint16_t microseconds) {
    status = CoprocStat_init_default;
    status.which_payload = CoprocStat_ultrasoundStat_tag;
    status.payload.ultrasoundStat.utsIndex = utsIndex;
    status.payload.ultrasoundStat.roundtripMicrosecs = microseconds;
    dispatcherEnqueueStatusFromISR(status);
}

void ultrasoundOnEchoEdge() {
    auto stampMicros = LL_TIM_GetCounter(utsTimer);
    if (!utsActiveIndex.has_value()) {
        return;
    }

    if (pinRead(utsEchoPin[*utsActiveIndex])) {
        risingEdgeMicros = stampMicros;
    } else {
        LL_EXTI_DisableIT_0_31(utsEchoPin[*utsActiveIndex].second);
        enqueueStatus(*utsActiveIndex, stampMicros - risingEdgeMicros);
        utsActiveIndex.reset();
        risingEdgeMicros = 0;
        LL_TIM_GenerateEvent_UPDATE(utsTimer); // Stop timer
        LL_TIM_ClearFlag_UPDATE(utsTimer);
    }
}

// Timer overflow signifies ultrasound timeout
extern "C" void UTSTIMER_HANDLER() {
    LL_TIM_ClearFlag_UPDATE(utsTimer);
    if (!utsActiveIndex.has_value()) {
        return;
    }

    LL_EXTI_DisableIT_0_31(utsEchoPin[*utsActiveIndex].second);
    enqueueStatus(*utsActiveIndex, 0);
    utsActiveIndex.reset();
}
