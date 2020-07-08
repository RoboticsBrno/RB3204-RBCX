#pragma once

#include "utils/Debug.hpp"
#include "utils/Regulator.hpp"

#include <algorithm>
#include <stdint.h>

inline const uint16_t motorLoopFreq = 100;

class Motor {
public:
    Motor()
        : m_velocityReg(INT16_MAX, 150000, 300000, 20000)
        , m_positionReg(INT16_MAX, 150000, 300000, 20000)
        , m_state(PowerCtrl) {}

    std::pair<int16_t, bool> poll(uint16_t encTicks) {
        int16_t actualTicksPerLoop = encTicks - m_lastEncTicks;
        m_actualPosition += actualTicksPerLoop;
        m_lastEncTicks = encTicks;
        if (m_state == PowerCtrl) {
            return std::make_pair(m_targetPower, false);
        } else if (m_state == BrakeCtrl) {
            return std::make_pair(m_targetBrakingPower, true);
        } else if (m_state == VelocityCtrl) {
            int16_t targetTicksPerLoop = m_targetVelocity / motorLoopFreq;
            uint16_t targetTicksRem = abs(m_targetVelocity % motorLoopFreq);
            if ((targetTicksRem * 4) / motorLoopFreq > m_dither) {
                targetTicksPerLoop += m_targetVelocity < 0 ? -1 : 1;
            }
            if (++m_dither >= 4) {
                m_dither = 0;
            }

            auto action
                = m_velocityReg.process(targetTicksPerLoop, actualTicksPerLoop);
            return std::make_pair(action, false);
        } else if (m_state == PositionCtrl) {
            auto action
                = m_positionReg.process(m_targetPosition, m_actualPosition);
            return std::make_pair(action, false);
        }
        return std::make_pair(0, false);
    }

    void setTargetPower(int16_t power) {
        if (m_state != PowerCtrl) {
            m_state = PowerCtrl;
        }
        m_targetPower = power;
    }

    void setTargetBrakingPower(int16_t brakingPower) {
        if (m_state != BrakeCtrl) {
            m_state = BrakeCtrl;
        }
        m_targetBrakingPower = brakingPower;
    }

    void setTargetVelocity(int16_t ticksPerSec) {
        if (m_state != VelocityCtrl) {
            m_velocityReg.clear();
            m_state = VelocityCtrl;
        }
        m_targetVelocity = ticksPerSec;
    }

    void homePosition(int32_t homedTicks) {
        m_targetPosition = homedTicks;
        m_actualPosition = homedTicks;
    }

    void setTargetPosition(int32_t ticks) {
        if (m_state != PositionCtrl) {
            m_positionReg.clear();
            m_state = PositionCtrl;
        }
        m_targetPosition = ticks;
    }

    void addTargetPosition(int32_t ticksDelta) {
        if (m_state != PositionCtrl) {
            m_positionReg.clear();
            m_state = PositionCtrl;
        }
        m_targetPosition += ticksDelta;
    }

    void setVelocityPid(uint32_t p, uint32_t i, uint32_t d) {
        m_velocityReg.setP(p);
        m_velocityReg.setI(i);
        m_velocityReg.setD(d);
        m_velocityReg.clear();
    }

    void setPositionPid(uint32_t p, uint32_t i, uint32_t d) {
        m_positionReg.setP(p);
        m_positionReg.setI(i);
        m_positionReg.setD(d);
        m_positionReg.clear();
    }

private:
    Regulator m_velocityReg;
    Regulator m_positionReg;
    int16_t m_targetPower;
    int16_t m_targetBrakingPower;
    int16_t m_targetVelocity;
    int32_t m_targetPosition;
    int32_t m_actualPosition;
    uint16_t m_dither;
    uint16_t m_lastEncTicks;
    enum MotorState {
        PowerCtrl,
        BrakeCtrl,
        VelocityCtrl,
        PositionCtrl,
    } m_state;
};
