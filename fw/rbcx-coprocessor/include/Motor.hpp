#pragma once

#include "utils/Debug.hpp"
#include "utils/Regulator.hpp"

#include <algorithm>
#include <stdint.h>

inline const uint16_t motorLoopFreq = 100;

enum class MotorState {
    PowerCtrl,
    BrakeCtrl,
    VelocityCtrl,
};

class Motor {
public:
    Motor()
        : m_velocityReg(SHRT_MAX, 150000, 300000, 20000)
        , m_state(MotorState::PowerCtrl) {}

    std::pair<int16_t, bool> poll(uint16_t encTicks) {
        int16_t actualTicksPerLoop = encTicks - m_lastEncTicks;
        m_lastEncTicks = encTicks;
        if (m_state == MotorState::PowerCtrl) {
            return std::make_pair(m_targetPower, false);
        } else if (m_state == MotorState::BrakeCtrl) {
            return std::make_pair(m_targetBrakingPower, true);
        } else if (m_state == MotorState::VelocityCtrl) {
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

            return std::make_pair(action, false);
        }
        return std::make_pair(0, false);
    }

    void setTargetPower(int16_t power) {
        if (m_state != MotorState::PowerCtrl) {
            m_state = MotorState::PowerCtrl;
        }
        m_targetPower = power;
    }

    void setTargetBrakingPower(int16_t brakingPower) {
        if (m_state != MotorState::BrakeCtrl) {
            m_state = MotorState::BrakeCtrl;
        }
        m_targetBrakingPower = brakingPower;
    }

    void setTargetVelocity(int16_t ticksPerSec) {
        if (m_state != MotorState::VelocityCtrl) {
            m_velocityReg.clear();
            m_state = MotorState::VelocityCtrl;
        }
        m_targetVelocity = ticksPerSec;
    }

    void setVelocityPid(uint32_t p, uint32_t i, uint32_t d) {
        m_velocityReg.P(p);
        m_velocityReg.I(i);
        m_velocityReg.D(d);
        m_velocityReg.clear();
    }

private:
    Regulator<int32_t> m_velocityReg;
    int16_t m_targetVelocity;
    int16_t m_targetPower;
    int16_t m_targetBrakingPower;
    uint16_t m_dither;
    uint16_t m_lastEncTicks;
    MotorState m_state;
};
