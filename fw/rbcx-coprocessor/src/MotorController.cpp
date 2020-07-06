#include "MotorController.hpp"
#include "Bsp.hpp"
#include "Motor.hpp"
#include "utils/Debug.hpp"
#include "utils/TaskWrapper.hpp"

#include "semphr.h"
#include "stm32f1xx_ll_tim.h"
#include <climits>
#include <cstdlib>

static void setPwmValue(TIM_TypeDef* timer, uint8_t motorIndex, uint16_t value);
static void setMotorPower(uint8_t motorIndex, int32_t power, bool brake);

static constexpr uint16_t maxPwm = 2000;
static std::array<Motor, 4> motor;
static TaskWrapper<1024> motorTask;
static void taskFunc();
static SemaphoreHandle_t motorSem;
static StaticSemaphore_t motorSemStruct;

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
    ocInit.CompareValue = 0;
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

    motorSem = xSemaphoreCreateBinaryStatic(&motorSemStruct);
    xSemaphoreGive(motorSem);
    motorTask.start("motors", 2, taskFunc);
}

static void taskFunc() {
    while (true) {
        auto now = xTaskGetTickCount();
        if (xSemaphoreTake(motorSem, pdMS_TO_TICKS(100) == pdTRUE)) {
            for (int m : { 0, 1, 2, 3 }) {
                uint16_t encTicks = LL_TIM_GetCounter(encoderTimer[m]);
                auto action = motor[m].poll(encTicks);
                setMotorPower(m, action.first, action.second);
            }
            xSemaphoreGive(motorSem);
        }
        vTaskDelayUntil(&now, pdMS_TO_TICKS(1000 / motorLoopFreq));
    }
}

void motorDispatch(const CoprocReq_MotorReq& request) {
    if (request.motorIndex > 3) {
        return;
    }

    auto& targetMotor = motor[request.motorIndex];

    if (xSemaphoreTake(motorSem, pdMS_TO_TICKS(100)) == pdTRUE) {
        switch (request.which_motorCmd) {
        case CoprocReq_MotorReq_setPower_tag:
            targetMotor.setTargetPower(request.motorCmd.setPower);
            break;
        case CoprocReq_MotorReq_setBrake_tag:
            targetMotor.setTargetBrakingPower(request.motorCmd.setBrake);
            break;
        case CoprocReq_MotorReq_setVelocity_tag: {
            auto ticksPerSec = request.motorCmd.setVelocity;
            if (ticksPerSec > SHRT_MAX || ticksPerSec < SHRT_MIN) {
                DEBUG("Motor %d target velocity out of range <-32768; 32767> "
                      "(%d).\n",
                    int(request.motorIndex), int(ticksPerSec));
                return;
            }
            targetMotor.setTargetVelocity(ticksPerSec);
        } break;
        case CoprocReq_MotorReq_setVelocityRegCoefs_tag: {
            auto& coefs = request.motorCmd.setVelocityRegCoefs;
            targetMotor.setVelocityPid(coefs.p, coefs.i, coefs.d);
        } break;
        }
        xSemaphoreGive(motorSem);
    }
}

void motorReset() {
    xSemaphoreTake(motorSem, portMAX_DELAY);
    for (int idx : { 0, 1, 2, 3 }) {
        motor[idx].setTargetPower(0);
        setMotorPower(idx, 0, false);
    }
    xSemaphoreGive(motorSem);
}

static void setPwmValue(
    TIM_TypeDef* timer, uint8_t motorIndex, uint16_t value) {
    reinterpret_cast<__IO uint16_t*>(&timer->CCR1)[motorIndex << 1] = value;
}

static void setMotorPower(uint8_t motorIndex, int32_t power, bool brake) {
    if (power > SHRT_MAX || power < SHRT_MIN) {
        DEBUG("Motor %d power out of range <-32768; 32767> (%d).\n",
            int(motorIndex), int(power));
        return;
    }

    uint16_t pwm = unsigned(abs(power) * maxPwm) / 32768;
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
