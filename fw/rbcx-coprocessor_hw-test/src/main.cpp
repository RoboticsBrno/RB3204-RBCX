#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_rtc.h"

#include <array>
#include "Bsp.hpp"
#include "CdcUartTunnel.hpp"
#include "UsbCdcLink.h"
#include "ControlUart.hpp"

#include "build_time.hpp"

#include <cstring>
#include <ctime>

void sendDebugStr(const char* str) {
    const size_t len = strlen(str);
    tunnelUartTx(reinterpret_cast<const uint8_t*>(str), len);
    HAL_Delay(1 + len / 11);
}

int main() {
    clocksInit();
    HAL_Init();
    pinsInit();
    tunnelUartInit();
    controlUartInit();    
    cdcLinkInit();
    HAL_Delay(1000);
    constexpr size_t debugBufferSize = 128;
    char debugBuffer[debugBufferSize] = { 0 };
    sendDebugStr("RBCX v1.0 HW test\n\t" __DATE__ " " __TIME__ "\n");
    HAL_PWR_EnableBkUpAccess();
    uint32_t isRtcValid = LL_RTC_BKP_GetRegister(BKP, LL_RTC_BKP_DR1);
    snprintf(debugBuffer, debugBufferSize, "isRtcValid %lu\n", isRtcValid);
    sendDebugStr(debugBuffer);
    if (isRtcValid == 0) {
        while (!LL_RTC_IsActiveFlag_RTOF(RTC));
        LL_RTC_DisableWriteProtection(RTC);
        //LL_PWR_EnableBkUpAccess();
        LL_RCC_LSE_Enable();
        uint32_t timeout = HAL_GetTick();
        while (!LL_RCC_LSE_IsReady()) {
            if ((HAL_GetTick() - timeout) > 1000) {
                sendDebugStr("Could not initialize 32.768 kHz Quartz.");
                break;
            }
        }
        LL_RTC_TIME_Set(RTC, UNIX_TIMESTAMP);
        LL_RTC_SetAsynchPrescaler(RTC, 32767);
        LL_RTC_SetOutputSource(BKP, LL_RTC_CALIB_OUTPUT_SECOND);
        LL_RTC_EnableWriteProtection(RTC);
        timeout = HAL_GetTick();
        while (!LL_RTC_IsActiveFlag_RTOF(RTC)) {
            if ((HAL_GetTick() - timeout) > 1000) {
                sendDebugStr("RTC synchronization timeout.");
                break;
            }
        }
        LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
        LL_RCC_EnableRTC();
        isRtcValid = 1;
        LL_RTC_BKP_SetRegister(BKP, LL_RTC_BKP_DR1, isRtcValid);
        snprintf(debugBuffer, debugBufferSize, "RTC updated by current compilation time %lu.\n", UNIX_TIMESTAMP);
        sendDebugStr(debugBuffer);
    }
    LL_RTC_WaitForSynchro(RTC);
    uint8_t leds = 0x01;
    const uint32_t ledPeriod = 500;
    uint32_t nextLedTime = ledPeriod;
    bool ledTest = true;
    bool usbConnected = false;
    while (true) {
        if (ledTest && HAL_GetTick() >= nextLedTime) {
            switch (leds) {
            case 0x01: leds = 0x03; break;
            case 0x30: leds = 0x01; break;
            default: leds <<= 1; break;
            }
            for (uint8_t i = 0; i != ledPin.size()-1; ++i)
                pinWrite(ledPin[i+1], leds & (1<<i));
            nextLedTime += ledPeriod;
        }
        if (isPressed(buttonOffPin)) {
            pinWrite(powerPin, 0);
            pinWrite(ledPins, 0);
            ledTest = false;
            if (isPressed(button4Pin) && isPressed(button2Pin)) {
                LL_RCC_ForceBackupDomainReset();
                sendDebugStr("Force BACKUP reset.\n");
            }
            sendDebugStr("Shutting down...\n");
            for(;;);
        } else if (isPressed(buttonOnPin)) {
            pinWrite(powerPin, 1);
            ledTest = true;
        }
        for (uint8_t i = 1; i != 5; ++i) {
            if (isPressed(buttonPin[i])) {
                ledTest = false;
                pinWrite(ledPin[i], 1);
            } else if (!ledTest) {
                pinWrite(ledPin[i], 0);
            }
        }
        pinWrite(buzzerPin, isPressed(button4Pin) && isPressed(button3Pin));
        bool v = pinRead(usbBusDetectionPin);
        if (v != usbConnected) {
            usbConnected = v;
            if (usbConnected) {
                sendDebugStr("USB connected\n");
            } else {
                sendDebugStr("USB disconnected\n");
            }
        }
        if (LL_RTC_IsActiveFlag_SEC(RTC)) {
            LL_RTC_ClearFlag_SEC(RTC);
            snprintf(debugBuffer, debugBufferSize, "Time %lu\n", LL_RTC_TIME_Get(RTC));
            sendDebugStr(debugBuffer);
        }
        cdcLinkPoll();
        tunnelPoll();
        std::array<uint8_t, 255> loopback;
        auto len = controlUartRxFrame(loopback.data(), loopback.size());
        if (len && controlUartTxReady()) {
            controlUartTxFrame(loopback.data(), len);
        }
    }
}

extern "C" void SysTick_Handler() {
    HAL_IncTick();
}
