#pragma once

class Esp32Manager {
    Esp32Manager(const Esp32Manager&) = delete;

public:
    Esp32Manager();
    ~Esp32Manager();

    void poll();

    void queueReset(bool bootloader = false);

    void onEnRising();
    void onSerialBreak(bool dtr, bool rst);

private:
    enum EnHolderType {
        EnSerialBreaks = 0,
        EnSwReset = 1,
    };

    enum QueuedReset {
        RstNone = 0,
        RstNormal = 1,
        RstBootloader = 2,
    };

    void holdReset(EnHolderType typ);
    void releaseReset(EnHolderType typ, bool strapForBootloader = false);

    void strapPins(bool bootloader);
    void unstrapPins();

    uint32_t m_unstrapAt;
    uint32_t m_checkBreaksAt;

    uint32_t m_enPinHolders;

    QueuedReset m_queuedReset;
    bool m_previousEnEdge;
    bool m_lastRts;
    bool m_lastDtr;
};

extern Esp32Manager sEsp32Manager;