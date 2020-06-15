#pragma once

#include "utils/QueueWrapper.hpp"
#include "utils/RingBuffer.hpp"
#include "utils/TaskWrapper.hpp"

// These methods are weakly linked as empty from Esp32Manager.cpp,
// override them near the code which uses these pins.
// * xxFreeUp - called when Esp32Manager needs to assume control of the pin,
//              you can clean-up here if needed.
// * xxRestore - called when Esp32Manager is done with the pins, you should call
//               your pinInits here.
void esp32Pin0FreeUp();
void esp32Pin0Restore();
void esp32Pin2FreeUp();
void esp32Pin2Restore();
void esp32Pin12FreeUp();
void esp32Pin12Restore();
void esp32Pin15FreeUp();
void esp32Pin15Restore();

class Esp32Manager {
    Esp32Manager(const Esp32Manager&) = delete;

public:
    Esp32Manager() {};
    ~Esp32Manager() {};

    void init();

    void queueReset(bool bootloader = false);

    void onEnRising();
    void onSerialBreak(bool dtr, bool rst);

private:
    enum NotificationType : uint32_t {
        NtfNone = 0,
        NtfResetNormal = (1 << 0),
        NtfResetBootloader = (1 << 1),
    };

    struct SerialBreak {
        TickType_t timestamp;
        bool dtr;
        bool rts;
    };

    void notifyExecutor(NotificationType state);

    void executorTask();
    void executeReset(bool bootloader);

    TaskWrapper<1024> m_task;
    RingBuffer<SerialBreak, 3> m_serialBreaks;
};

extern Esp32Manager sEsp32Manager;
