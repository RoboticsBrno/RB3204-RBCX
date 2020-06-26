#pragma once

#include "utils/Debug.hpp"
#include "utils/Regulator.hpp"

#include <algorithm>
#include <stdint.h>

inline const uint16_t motorLoopPeriodMs = 10;

using VelocityReg = Regulator<int32_t>;

enum class MotorState {
    PowerCtrl,
    VelocityCtrl,
};

class Motor {
public:
    Motor()
        : m_velocityReg(USHRT_MAX, 5'000'000, 5'000'000, 0) {}

    std::pair<int16_t, bool> poll(uint16_t encTicks) {
        int16_t velocity = encTicks - m_lastEncTicks;
        auto action = m_velocityReg.process(velocity);

        DEBUG("Pos:%d Vel:%d (W:%d E:%d I:%d X:%d)\n", int(encTicks),
            int(velocity), int(m_velocityReg()), int(m_velocityReg.e()),
            int(m_velocityReg.integrator()), int(m_velocityReg.x()));
        m_lastEncTicks = encTicks;
        return std::make_pair(action, false);
    }

    void setTargetVelocity(int32_t edgesPerSec) {
        int32_t edgesPerLoop = (edgesPerSec * motorLoopPeriodMs) / 1000;
        m_velocityReg(edgesPerLoop);
    }

    void setVelocityPid(uint32_t p, uint32_t i, uint32_t d) {
        m_velocityReg.P(p);
        m_velocityReg.I(i);
        m_velocityReg.D(d);
    }

private:
    Regulator<int32_t> m_velocityReg;
    uint16_t m_lastEncTicks;
};
