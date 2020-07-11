#pragma once

#include "utils/Debug.hpp"
#include "utils/Regulator.hpp"

#include <stdint.h>

inline const uint16_t motorLoopFreq = 100;

class Motor {
public:
    Motor()
        : m_velocityReg(INT16_MAX, 150000, 300000, 20000)
        , m_positionReg(500, 1000, 0, 0) {}

    bool atTargetPosition() const {
        return uint32_t(abs(m_actualPosition - m_targetPosition))
            <= m_posEpsilon;
    }

    bool atStandstill() const {
        return uint32_t(abs(m_actualTicksPerLoop)) <= m_velEpsilon;
    }

    MotorMode mode() const { return m_mode; }
    void modeChange(MotorMode newMode) { m_mode = newMode; }

    void reportStat(CoprocStat_MotorStat& stat) {
        stat.mode = m_mode;
        stat.position = m_actualPosition;
        stat.power = m_actualPower;
        stat.velocity = m_actualTicksPerLoop * motorLoopFreq;
    };

    int16_t poll(uint16_t encTicks) {
        m_actualTicksPerLoop = encTicks - m_lastEncTicks;
        m_actualPosition += m_actualTicksPerLoop;
        m_lastEncTicks = encTicks;

        switch (m_mode) {
        case MotorMode_POWER:
        case MotorMode_BRAKE:
            break;
        case MotorMode_POSITION:
        case MotorMode_POSITION_IDLE: {
            if (atTargetPosition() && atStandstill()) {
                m_positionReg.clear();
                m_targetVelocity = 0;
                modeChange(MotorMode_POSITION_IDLE);
                break;
            }
            auto action
                = m_positionReg.process(m_targetPosition, m_actualPosition);

            // Limit ramp-up to max acceleration
            if (action > m_targetVelocity + m_maxAccel) {
                m_targetVelocity += m_maxAccel;
            } else if (action < m_targetVelocity - m_maxAccel) {
                m_targetVelocity -= m_maxAccel;
            } else {
                m_targetVelocity = action;
            }
        }
        // fallthrough
        case MotorMode_VELOCITY: {
            int16_t targetTicksPerLoop = m_targetVelocity / motorLoopFreq;
            uint16_t targetTicksRem = abs(m_targetVelocity % motorLoopFreq);
            if ((targetTicksRem * 4) / motorLoopFreq > m_dither) {
                targetTicksPerLoop += sgn m_targetVelocity < 0 ? -1 : 1;
            }
            if (++m_dither >= 4) {
                m_dither = 0;
            }

            auto action = m_velocityReg.process(
                targetTicksPerLoop, m_actualTicksPerLoop);
            m_actualPower = action;
        } break;
        default:
            abort();
        }
        return m_actualPower;
    }

    void setTargetPower(int16_t power) {
        if (m_mode != MotorMode_POWER) {
            m_mode = MotorMode_POWER;
        }
        m_actualPower = power;
    }

    void setTargetBrakingPower(int16_t brakingPower) {
        if (m_mode != MotorMode_BRAKE) {
            modeChange(MotorMode_BRAKE);
        }
        m_actualPower = brakingPower;
    }

    void setTargetVelocity(int16_t ticksPerSec) {
        if (m_mode != MotorMode_VELOCITY) {
            m_velocityReg.clear();
            modeChange(MotorMode_VELOCITY);
        }
        m_targetVelocity = ticksPerSec;
    }

    void homePosition(int32_t homedTicks) {
        m_targetPosition = homedTicks;
        m_actualPosition = homedTicks;
    }

    void setTargetPosition(
        const CoprocReq_MotorReq_SetPosition& req, bool additive) {
        if (m_mode != MotorMode_POSITION) {
            m_positionReg.clear();
            modeChange(MotorMode_POSITION);
        }
        if (additive) {
            m_targetPosition += req.targetPosition;
        } else {
            m_targetPosition = req.targetPosition;
        }
        m_positionReg.setMaxOutput(req.runningVelocity);
    }

    void setVelocityPid(const RegCoefs& coefs) {
        m_velocityReg.setP(coefs.p);
        m_velocityReg.setI(coefs.i);
        m_velocityReg.setD(coefs.d);
        m_velocityReg.clear();
    }

    void setPositionPid(const RegCoefs& coefs) {
        m_positionReg.setP(coefs.p);
        m_positionReg.setI(coefs.i);
        m_positionReg.setD(coefs.d);
        m_positionReg.clear();
    }

    void setConfig(const MotorConfig& config) {
        m_velEpsilon = config.velEpsilon;
        m_posEpsilon = config.posEpsilon;
        m_maxAccel = config.maxAccel / motorLoopFreq;
    }

private:
    Regulator m_velocityReg;
    Regulator m_positionReg;
    int16_t m_actualPower;
    int16_t m_targetVelocity;
    int32_t m_targetPosition;
    int32_t m_actualPosition;
    int16_t m_actualTicksPerLoop;
    uint16_t m_dither;
    uint16_t m_lastEncTicks;
    uint16_t m_posEpsilon = 3;
    uint16_t m_velEpsilon = 3;
    uint16_t m_maxAccel = 2000 / motorLoopFreq;
    MotorMode m_mode = MotorMode_POWER;
};
