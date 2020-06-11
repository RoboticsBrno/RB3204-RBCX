#include "stm32f1xx_ll_utils.h"

#include "Bsp.hpp"
#include "FreeRTOS.h"
#include "task.h"

static StaticTask_t idleTaskStruct;
static StackType_t idleTaskStack[configMINIMAL_STACK_SIZE];

extern "C" void vApplicationGetIdleTaskMemory(
    StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer,
    uint32_t* pulIdleTaskStackSize) {
    *ppxIdleTaskTCBBuffer = &idleTaskStruct;
    *ppxIdleTaskStackBuffer = idleTaskStack;
    *pulIdleTaskStackSize = sizeof(idleTaskStack) / sizeof(StackType_t);
}

#if configUSE_TIMERS
static StaticTask_t timerTaskStruct;
static StackType_t timerTaskStack[configTIMER_TASK_STACK_DEPTH];

extern "C" void vApplicationGetTimerTaskMemory(
    StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer,
    uint32_t* pulIdleTaskStackSize) {
    *ppxIdleTaskTCBBuffer = &timerTaskStruct;
    *ppxIdleTaskStackBuffer = timerTaskStack;
    *pulIdleTaskStackSize = sizeof(timerTaskStack) / sizeof(StackType_t);
}
#endif

extern "C" void vApplicationStackOverflowHook(
    TaskHandle_t xTask, signed char* pcTaskName) {
    setLeds(0xFFFFFFFF);

    printf("Stack overflow!\n");
    printf("Task: %s\n", pcTaskName);

    while (true) {
        LL_mDelay(200);
        setLeds(0);
        LL_mDelay(200);
        setLeds(0xFFFFFFFF);
    }
}
