#include "UltrasoundController.hpp"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_tim.h"
#include "utils/Debug.hpp"
#include "utils/QueueWrapper.hpp"
#include "utils/TaskWrapper.hpp"
#include <optional>

#include "Bsp.hpp"
#include "Dispatcher.hpp"

static const uint16_t pingTimeoutMs = 30;
static const uint16_t echoTimeoutMs = 200;
static CoprocStat status;
static int utsActiveIndex;
static uint16_t risingEdgeMicros;
static TaskWrapper<512> utsTask;
static QueueWrapper<int, 16> trigQueue;

static void enqueueStatus(int utsIndex, uint16_t microseconds) {
    status = CoprocStat_init_default;
    status.which_payload = CoprocStat_ultrasoundStat_tag;
    status.payload.ultrasoundStat.utsIndex = utsIndex;
    status.payload.ultrasoundStat.roundtripMicrosecs = microseconds;
    dispatcherEnqueueStatus(status);
}

void ultrasoundInit() {
    LL_TIM_InitTypeDef timInit;
    LL_TIM_StructInit(&timInit);
    auto apb1TimClk = 2 * HAL_RCC_GetPCLK1Freq();

    timInit.Autoreload = pingTimeoutMs * 1000;
    timInit.Prescaler = apb1TimClk / 1'000'000; // 1 us ticks
    LL_TIM_Init(utsTimer, &timInit);
    LL_TIM_SetOnePulseMode(utsTimer, LL_TIM_ONEPULSEMODE_SINGLE);
    LL_TIM_SetUpdateSource(utsTimer, LL_TIM_UPDATESOURCE_COUNTER);
    LL_TIM_ClearFlag_UPDATE(utsTimer);

    LL_EXTI_DisableIT_0_31(utsEchoPins.second);

    trigQueue.create();
    utsTask.start("ultrasound", 2, []() {
        while (true) {
            int utsIndex;
            trigQueue.pop_front(utsIndex);

            // Wait for potential ECHO high
            while (pinRead(utsEchoPin[utsIndex])) {
                vTaskDelay(pdMS_TO_TICKS(5));
            }
            if (pinRead(utsEchoPin[utsIndex])) {
                DEBUG("ECHO %d hanging high", utsIndex);
                continue;
            }

            LL_TIM_GenerateEvent_UPDATE(utsTimer);
            LL_TIM_ClearFlag_UPDATE(utsTimer);

            // Perform TRIG pulse
            taskENTER_CRITICAL();
            LL_TIM_EnableCounter(utsTimer);
            pinWrite(utsTrigPin[utsIndex], true);
            while (LL_TIM_GetCounter(utsTimer) < 10
                && !LL_TIM_IsActiveFlag_UPDATE(utsTimer)) { // 10 us pulse
            }
            pinWrite(utsTrigPin[utsIndex], false);
            taskEXIT_CRITICAL();

            // Wait for ECHO measurement from ISR
            utsActiveIndex = utsIndex;
            LL_EXTI_EnableIT_0_31(utsEchoPin[utsIndex].second);
            uint32_t micros;
            auto success = xTaskNotifyWait(
                0U, ~0U, &micros, pdMS_TO_TICKS(pingTimeoutMs));
            LL_EXTI_DisableIT_0_31(utsEchoPin[utsIndex].second);
            if (success) {
                enqueueStatus(utsIndex, micros);
            } else {
                enqueueStatus(utsIndex, 0);
            }
        }
    });
}

void ultrasoundDispatch(const CoprocReq_UltrasoundReq& request) {
    if (request.utsIndex > 3) {
        DEBUG("Ultrasound index %lu out of range\n", request.utsIndex);
        return;
    }

    switch (request.which_utsCmd) {
    case CoprocReq_UltrasoundReq_singlePing_tag:
        if (!trigQueue.push_back(request.utsIndex, 0)) {
            DEBUG("Ultrasound TRIG queue overflow\n");
        }
        break;
    }
}

void ultrasoundOnEchoEdge() {
    auto stampMicros = LL_TIM_GetCounter(utsTimer);

    if (pinRead(utsEchoPin[utsActiveIndex])) {
        risingEdgeMicros = stampMicros;
    } else if (risingEdgeMicros > 0) {
        uint32_t deltaMicros = stampMicros - risingEdgeMicros;
        BaseType_t woken = 0;
        xTaskNotifyFromISR(
            utsTask.handle(), deltaMicros, eSetValueWithOverwrite, &woken);
        portYIELD_FROM_ISR(woken);
    }
}
