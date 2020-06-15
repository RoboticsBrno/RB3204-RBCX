#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

#include "FreeRTOS.h"
#include "task.h"

#include "Bsp.hpp"
#include "utils/Debug.hpp"

#include "Esp32Manager.hpp"

Esp32Manager sEsp32Manager;

template <void (*FreeUp)(), void (*Restore)()> class PinGuard {
public:
    PinGuard() { (*FreeUp)(); }
    ~PinGuard() { (*Restore)(); }
};

#define PIN_GUARD(PIN_NUM)                                                     \
    PinGuard<&esp32Pin##PIN_NUM##FreeUp, &esp32Pin##PIN_NUM##Restore>          \
        _pin##PIN_NUM##guard;

void __attribute__((weak)) esp32Pin0FreeUp() {}
void __attribute__((weak)) esp32Pin0Restore() {
    pinInit(esp0Pin, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
}
void __attribute__((weak)) esp32Pin2FreeUp() {}
void __attribute__((weak)) esp32Pin2Restore() {
    pinInit(esp2Pin, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
}
void __attribute__((weak)) esp32Pin12FreeUp() {}
void __attribute__((weak)) esp32Pin12Restore() {
    pinInit(esp12Pin, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
}
void __attribute__((weak)) esp32Pin15FreeUp() {}
void __attribute__((weak)) esp32Pin15Restore() {
    pinInit(esp15Pin, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
}

static void esp32PinENFreeUp() {
    pinInit(espEnPin, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
}

static void esp32PinENRestore() {
    pinInit(espEnPin, GPIO_MODE_IT_RISING, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW);
}

void Esp32Manager::init() {
    // Keep ESP32 in reset until we start up
    esp32PinENFreeUp();
    pinWrite(espEnPin, 0);

    m_task.start(
        "esp32", configMAX_PRIORITIES - 1, [this]() { executorTask(); });
}

void Esp32Manager::notifyExecutor(NotificationType state) {
    if (isInInterrupt()) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(
            m_task.handle(), state, eSetBits, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } else {
        xTaskNotify(m_task.handle(), state, eSetBits);
    }
}

void Esp32Manager::queueReset(bool bootloader) {
    notifyExecutor(!bootloader ? NtfResetNormal : NtfResetBootloader);
}

void Esp32Manager::executorTask() {
    uint32_t notification = 0;

    executeReset(false);

    while (true) {
        xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, &notification, portMAX_DELAY);
        if (notification & NtfResetNormal) {
            executeReset(false);
        } else if (notification & NtfResetBootloader) {
            executeReset(true);
        }
    }
}

void Esp32Manager::executeReset(bool bootloader) {
    PIN_GUARD(EN);
    pinWrite(espEnPin, 0);

    if (bootloader) {
        DEBUG("Resetting ESP32 into bootloader...\n");
    } else {
        DEBUG("Resetting ESP32 into user app...\n");
    }

    // min 150uS to reset
    vTaskDelay(1);

    {
        PIN_GUARD(0);
        PIN_GUARD(2);
        PIN_GUARD(12);
        PIN_GUARD(15);

        pinInit(esp0Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
        pinInit(
            esp12Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
        pinInit(
            esp15Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);

        pinWrite(esp12Pin, 0); // 3.3v flash
        pinWrite(esp15Pin, 1); // Do not silence bootloader messages

        if (!bootloader) {
            pinWrite(esp0Pin, 1); // normal
        } else {
            pinWrite(esp0Pin, 0); // bootloader

            pinInit(
                esp2Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
            pinWrite(esp2Pin, 0); // bootloader confirm
        }

        pinWrite(espEnPin, 1);

        // Hold time of bootstrap pins is 1ms
        vTaskDelay(1);
    }

    DEBUG("Reset done.\n");
}

void Esp32Manager::onEnRising() { queueReset(false); }

void Esp32Manager::onSerialBreak(bool dtr, bool rts) {
    m_serialBreaks.push(SerialBreak {
        .timestamp = xTaskGetTickCount(),
        .dtr = dtr,
        .rts = rts,
    });

    // https://github.com/espressif/esptool/blob/4a1e87d290e9ba2870b4d092baf7b9ae15e4095d/esptool.py#L469

    // Bootloader
    // [   1441314][src/UsbCdcLink.c:205]: CONTROL_LINE_STATE DTR 0 RTS 1 <-- 0
    // [   1441414][src/UsbCdcLink.c:205]: CONTROL_LINE_STATE DTR 1 RTS 1 <-- 1
    // [   1441415][src/UsbCdcLink.c:205]: CONTROL_LINE_STATE DTR 1 RTS 0 <-- 2
    // [   1441465][src/UsbCdcLink.c:205]: CONTROL_LINE_STATE DTR 0 RTS 0

    // Reboot
    //                                    (don't care)          DTR 0       <-- 0
    // [    444506][src/UsbCdcLink.cpp:205]: CONTROL_LINE_STATE DTR 0 RTS 1 <-- 1
    // [    444607][src/UsbCdcLink.cpp:205]: CONTROL_LINE_STATE DTR 0 RTS 0 <-- 2

    const auto& b = m_serialBreaks;
    if (b[0].timestamp == 0) {
        return;
    }

    // Detect RTS High -> low between 1 and 2
    if (!b[1].rts || b[2].rts) {
        return;
    }

    // minimum 50ms, max 2s
    const auto start = b[2].dtr ? b[0].timestamp : b[1].timestamp;
    const auto diff = b[2].timestamp - start;
    if (diff < pdMS_TO_TICKS(50) || diff > pdMS_TO_TICKS(2000)) {
        return;
    }

    // We're resetting now. Bootloader or Normal?
    queueReset(b[2].dtr);
}
