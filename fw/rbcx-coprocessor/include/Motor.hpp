#pragma once

#include "utils/Debug.hpp"
#include "utils/Regulator.hpp"

#include <stdint.h>
#include <tuple>

inline const uint16_t motorLoopPeriodMs = 40;

using VelocityReg = Regulator<int32_t>;

enum class MotorState {
    PowerCtrl,
    VelocityCtrl,
};

class Motor {
public:
    std::tuple<int16_t, bool> poll(uint16_t encTicks) {
        int16_t velocity = encTicks - m_lastEncTicks;
        auto action = m_velocityReg.process(velocity);

        DEBUG("Pos:%d Vel:%d (W:%d E:%d I:%d X:%d)\n", int(encTicks),
            int(velocity), int(m_velocityReg()), int(m_velocityReg.e()),
            int(m_velocityReg.integrator()), int(m_velocityReg.x()));
        m_lastEncTicks = encTicks;
        return std::make_tuple(action, false);
    }

    void setTargetVelocity(int32_t edgesPerSec) {
        int32_t edgesPerLoop = edgesPerSec / motorLoopPeriodMs;
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
