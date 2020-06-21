#include "MotorController.hpp"
#include "Bsp.hpp"
#include "utils/Debug.hpp"

#include "stm32f1xx_ll_tim.h"
#include <cmath>

static void setMotorPower(uint8_t motorIndex, float power, bool brake);
static constexpr uint16_t maxPwm = 2000;

void motorInit() {
    LL_TIM_InitTypeDef pwmInit;
    LL_TIM_StructInit(&pwmInit);

    pwmInit.Prescaler = 0;
    // this sets interrupts flag when counter reachs TOP:
    pwmInit.CounterMode = LL_TIM_COUNTERMODE_CENTER_DOWN;
    pwmInit.Autoreload = maxPwm;
    pwmInit.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    pwmInit.RepetitionCounter = 0;

    LL_TIM_OC_InitTypeDef ocInit;
    LL_TIM_OC_StructInit(&ocInit);
    ocInit.OCMode = LL_TIM_OCMODE_PWM2;
    ocInit.OCState = LL_TIM_OCSTATE_ENABLE;
    ocInit.OCNState = LL_TIM_OCSTATE_ENABLE;
    ocInit.CompareValue = maxPwm / 2;
    ocInit.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    ocInit.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
    ocInit.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
    ocInit.OCNIdleState = LL_TIM_OCIDLESTATE_HIGH;

    LL_TIM_Init(pwmTimer, &pwmInit);
    for (uint16_t channel = LL_TIM_CHANNEL_CH1; channel != 0; channel <<= 4) {
        LL_TIM_OC_Init(pwmTimer, channel, &ocInit);
        LL_TIM_OC_EnablePreload(pwmTimer, channel);
    }
    LL_TIM_SetOffStates(pwmTimer, LL_TIM_OSSI_DISABLE, LL_TIM_OSSR_ENABLE);
    LL_TIM_GenerateEvent_UPDATE(pwmTimer);
    LL_TIM_EnableAllOutputs(pwmTimer);
    LL_TIM_EnableCounter(pwmTimer);

    for (int motorIndex : { 0, 1, 2, 3 }) {
        setMotorPower(motorIndex, 0, false);
    }

    LL_TIM_ENCODER_InitTypeDef encInit;
    encInit.EncoderMode = LL_TIM_ENCODERMODE_X4_TI12;
    encInit.IC1Polarity = LL_TIM_IC_POLARITY_RISING;
    encInit.IC1ActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI;
    encInit.IC1Prescaler = LL_TIM_ICPSC_DIV1;
    encInit.IC1Filter = LL_TIM_IC_FILTER_FDIV1;
    encInit.IC2Polarity = LL_TIM_IC_POLARITY_RISING;
    encInit.IC2ActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI;
    encInit.IC2Prescaler = LL_TIM_ICPSC_DIV1;
    encInit.IC2Filter = LL_TIM_IC_FILTER_FDIV1;
    for (auto timer : encoderTimer) {
        LL_TIM_ENCODER_Init(timer, &encInit);
        LL_TIM_EnableCounter(timer);
    }
}

void motorDispatch(const CoprocReq_MotorReq& request) {
    if (request.motorIndex > 3) {
        return;
    }

    switch (request.which_motorCmd) {
    case CoprocReq_MotorReq_setPwm_tag:
        if (request.motorCmd.setPwm > 1 || request.motorCmd.setPwm < -1) {
            DEBUG("Motor %d power out of range <-1; 1> (%dE-3).\n",
                int(request.motorIndex), int(request.motorCmd.setPwm * 1000));
            return;
        }
        setMotorPower(request.motorIndex, request.motorCmd.setPwm, false);
        break;
    }
}

static void setPwmValue(TIM_TypeDef* timer, uint8_t channel, uint16_t value) {
    reinterpret_cast<__IO uint16_t*>(&timer->CCR1)[channel << 1] = value;
}

static void setMotorPower(uint8_t motorIndex, float power, bool brake) {
    uint16_t pwm = fabsf(power) * maxPwm;
    setPwmValue(pwmTimer, motorIndex, pwm);
    if (pwm == 0 || brake) {
        switch (motorIndex) {
        case 3:
            IN4PORT->BRR = IN4AMASK | IN4BMASK; // set LOW on IN4A and IN4B
            pwmTimer->CCER |= TIM_CCER_CC4P; // invert channel 4
            break;
        default:
            // set PWM on both channels and invert positive channel
            pwmTimer->CCER
                = (pwmTimer->CCER & ~(TIM_CCER_CC1NP << (motorIndex << 2)))
                | (TIM_CCER_CC1E << (motorIndex << 2))
                | (TIM_CCER_CC1NE << (motorIndex << 2))
                | (TIM_CCER_CC1P << (motorIndex << 2));
            break;
        }
    } else {
        switch (motorIndex) {
        case 3:
            IN4PORT->BSRR = power < 0
                ? IN4AMASK | (IN4BMASK << 16) // pinWrite(in4aPin, power < 0);
                : IN4BMASK | (IN4AMASK << 16); // pinWrite(in4bPin, power > 0);
            pwmTimer->CCER &= ~TIM_CCER_CC4P; // make channel 4 non-inverted
            break;
        default:
            if (power > 0) {
                // set HIGH on positive channel and inverted PWM on negative channel
                pwmTimer->CCER
                    = (pwmTimer->CCER
                          & ~((TIM_CCER_CC1P << (motorIndex << 2))
                              | (TIM_CCER_CC1NE << (motorIndex << 2))))
                    | (TIM_CCER_CC1NP << (motorIndex << 2))
                    | (TIM_CCER_CC1E << (motorIndex << 2));
            } else {
                // set HIGH on negative channel and inverted PWM on positive channel
                pwmTimer->CCER
                    = (pwmTimer->CCER
                          & ~((TIM_CCER_CC1E << (motorIndex << 2))
                              | (TIM_CCER_CC1NP << (motorIndex << 2))))
                    | (TIM_CCER_CC1NE << (motorIndex << 2))
                    | (TIM_CCER_CC1P << (motorIndex << 2));
            }
            break;
        }
    }
}
