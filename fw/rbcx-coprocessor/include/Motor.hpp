#pragma once

#include "utils/Debug.hpp"
#include "utils/Regulator.hpp"

#include <math.h>
#include <stdint.h>

inline const uint16_t motorLoopFreq = 100;

class Motor {
public:
    Motor() {
        m_lastEncTicks = 0;
        reset();
    }

    void reset() {
        m_velocityReg = Regulator(INT16_MAX, 150000, 300000, 20000);
        m_dither = 0;
        m_targetVelocity = 0;
        m_rampingVelocity = 0;
        m_actualPower = 0;
        m_actualPosition = 0;
        m_targetPosition = 0;
        m_actualTicksPerLoop = 0;
        m_posEpsilon = 3;
        m_velEpsilon = 3;
        m_maxAccel = 2000 / motorLoopFreq;
        m_mode = MotorMode_POWER;
    }

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
        case MotorMode_POSITION:
        case MotorMode_POSITION_IDLE: {
            if (atTargetPosition() && atStandstill()) {
                m_rampingVelocity = 0;
                m_targetVelocity = 0;
                modeChange(MotorMode_POSITION_IDLE);
            } else {
                int32_t posDelta = m_targetPosition - m_actualPosition;
                int16_t terminalVelocity = std::min<int32_t>(INT16_MAX,
                    sqrt(2 * motorLoopFreq * int32_t(m_maxAccel)
                        * abs(posDelta)));

                int16_t targetVelocity = std::min<int32_t>(
                    abs(m_targetVelocity), terminalVelocity);

                m_targetVelocity
                    = posDelta < 0 ? -targetVelocity : targetVelocity;
                // DEBUG("Vter:%d Vt:%d pd:%d St:%d\n", terminalVelocity,
                //     int(m_targetVelocity), posDelta, m_targetPosition);
            }
        } // fallthrough
        case MotorMode_VELOCITY: {
            int16_t accel = std::clamp<int32_t>(
                m_targetVelocity - m_rampingVelocity, -m_maxAccel, m_maxAccel);

            m_rampingVelocity += accel;

            int16_t targetTicksPerLoop = m_rampingVelocity / motorLoopFreq;
            uint16_t targetTicksRem = abs(m_rampingVelocity % motorLoopFreq);
            if ((targetTicksRem * 4) / motorLoopFreq > m_dither) {
                targetTicksPerLoop += m_rampingVelocity < 0 ? -1 : 1;
            }
            if (++m_dither >= 4) {
                m_dither = 0;
            }

            m_actualPower = m_velocityReg.process(
                targetTicksPerLoop, m_actualTicksPerLoop);
        } break;
        default:
            break;
        }
        return m_actualPower;
    }

    void setTargetPower(int16_t power) {
        if (m_mode != MotorMode_POWER) {
            modeChange(MotorMode_POWER);
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
            modeChange(MotorMode_POSITION);
        }
        if (additive) {
            m_targetPosition += req.targetPosition;
        } else {
            m_targetPosition = req.targetPosition;
        }
        m_targetVelocity = req.runningVelocity;
    }

    void setVelocityPid(const RegCoefs& coefs) {
        m_velocityReg.setP(coefs.p);
        m_velocityReg.setI(coefs.i);
        m_velocityReg.setD(coefs.d);
        m_velocityReg.clear();
    }

    void setConfig(const MotorConfig& config) {
        m_velEpsilon = config.velEpsilon;
        m_posEpsilon = config.posEpsilon;
        m_maxAccel = config.maxAccel / motorLoopFreq;
    }

private:
    Regulator m_velocityReg;
    int16_t m_actualPower;
    int16_t m_targetVelocity;
    int16_t m_rampingVelocity;
    int32_t m_targetPosition;
    int32_t m_actualPosition;
    int16_t m_actualTicksPerLoop;
    uint16_t m_dither;
    uint16_t m_lastEncTicks;
    uint16_t m_posEpsilon;
    uint16_t m_velEpsilon;
    uint16_t m_maxAccel;
    MotorMode m_mode;
};
