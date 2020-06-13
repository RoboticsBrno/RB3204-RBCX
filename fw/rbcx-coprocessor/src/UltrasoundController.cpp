#include "UltrasoundController.hpp"
#include "Bsp.hpp"
#include "Dispatcher.hpp"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_tim.h"

void ultrasoundInit() {
    LL_TIM_InitTypeDef init;
    LL_TIM_StructInit(&init);
    auto apb1TimClk = 2 * HAL_RCC_GetPCLK1Freq();

    init.Autoreload = 30'000; // 30 ms overflow
    init.Prescaler = apb1TimClk / 1'000'000; // 1 us ticks
    LL_TIM_Init(utsTimer, &init);
    LL_TIM_SetOnePulseMode(utsTimer, LL_TIM_ONEPULSEMODE_SINGLE);
    LL_TIM_SetUpdateSource(utsTimer, LL_TIM_UPDATESOURCE_COUNTER);

    for (auto irqn : utsEchoIRQns) {
        HAL_NVIC_SetPriority(irqn, utsIRQPrio, 0);
        HAL_NVIC_EnableIRQ(irqn);
    }

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
        LL_TIM_GenerateEvent_UPDATE(utsTimer);
        LL_TIM_EnableCounter(utsTimer);

        pinWrite(utsTrigPin[request.utsIndex], true);
        while (LL_TIM_GetCounter(utsTimer) < 10) { // 10 us pulse
        }
        pinWrite(utsTrigPin[request.utsIndex], false);

        LL_TIM_ClearFlag_UPDATE(utsTimer);
        LL_TIM_EnableIT_UPDATE(utsTimer);
        break;
    }
}

static CoprocStat status;

extern "C" void EXTI0_IRQHandler() {}
extern "C" void EXTI1_IRQHandler() {}
extern "C" void EXTI2_IRQHandler() {}
extern "C" void EXTI3_IRQHandler() {}

// Timer overflow signifies ultrasound timeout
extern "C" void UTSTIMER_HANDLER() {
    LL_TIM_DisableIT_UPDATE(utsTimer);
    LL_TIM_ClearFlag_UPDATE(utsTimer);
    status = CoprocStat_init_default;
    status.which_payload = CoprocStat_ultrasoundStat_tag;
    status.payload.ultrasoundStat.roundtripMicrosecs = 0;
    dispatcherEnqueueStatusFromISR(status);
}
