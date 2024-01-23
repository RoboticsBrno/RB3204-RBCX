#pragma once

#include "FreeRTOS.h"
#include "semphr.h"

class BinarySemaphoreWrapper {
public:
    BinarySemaphoreWrapper()
        : m_handle(nullptr) {}

    ~BinarySemaphoreWrapper() {
        if (m_handle)
            vSemaphoreDelete(m_handle);
    }

    void create() {
        if (m_handle)
            abort();

        m_handle = xSemaphoreCreateBinaryStatic(&m_buffer);
    }

    SemaphoreHandle_t native_handle() const { return m_handle; }

    bool take(TickType_t timeout = portMAX_DELAY) {
        return xSemaphoreTake(m_handle, timeout) == pdTRUE;
    }

    void give() { xSemaphoreGive(m_handle); }

    void giveFromIsr() {
        BaseType_t woken = 0;
        xSemaphoreGiveFromISR(m_handle, &woken);
        portYIELD_FROM_ISR(woken);
    }

private:
    SemaphoreHandle_t m_handle;
    StaticSemaphore_t m_buffer;
};
