#pragma once

#include "utils/Debug.hpp"
#include "utils/Regulator.hpp"
#include "utils/XorShift.hpp"

#include <algorithm>
#include <stdint.h>

inline const uint16_t motorLoopFreq = 100;

using VelocityReg = Regulator<int32_t>;

enum class MotorState {
    PowerCtrl,
    VelocityCtrl,
};

class Motor {
public:
    Motor()
        : m_velocityReg(SHRT_MAX, 300000, 500000, 100000) {}

    std::pair<int16_t, bool> poll(uint16_t encTicks) {
        int16_t actualTicksPerLoop = encTicks - m_lastEncTicks;
        int16_t targetTicksPerLoop = m_targetVelocity / motorLoopFreq;
        int16_t targetTicksRem = abs(m_targetVelocity % motorLoopFreq);
        int16_t dither = abs(m_dither % motorLoopFreq);
        if ((targetTicksRem * 4) / motorLoopFreq > dither) {
            targetTicksPerLoop += m_targetVelocity < 0 ? -1 : 1;
        }
        if (++m_dither >= 4) {
            m_dither = 0;
        }

        m_velocityReg(targetTicksPerLoop);
        auto action = m_velocityReg.process(actualTicksPerLoop);

        DEBUG("Pos:%d Vel:%d (W:%d E:%d I:%d X:%d)\n", int(encTicks),
            int(actualTicksPerLoop), int(m_velocityReg()),
            int(m_velocityReg.e()), int(m_velocityReg.integrator()),
            int(m_velocityReg.x()));
        m_lastEncTicks = encTicks;
        return std::make_pair(action, false);
    }

    void setTargetVelocity(int16_t ticksPerSec) {
        m_targetVelocity = ticksPerSec;
    }

    void setVelocityPid(uint32_t p, uint32_t i, uint32_t d) {
        m_velocityReg.P(p);
        m_velocityReg.I(i);
        m_velocityReg.D(d);
    }

private:
    Regulator<int32_t> m_velocityReg;
    uint16_t m_dither;
    int16_t m_targetVelocity;
    uint16_t m_lastEncTicks;
};
