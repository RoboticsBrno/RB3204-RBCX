#include "MpuController.hpp"
#include "Dispatcher.hpp"
#include "I2cController.hpp"
#include "MpuDmpCode.hpp"
#include "MpuDriver.hpp"
#include "utils/TickTimer.hpp"

#include "Bsp.hpp"

#include "utils/QueueWrapper.hpp"
#include "utils/TaskWrapper.hpp"

#include "FreeRTOS.h"
#include "timers.h"

#include "event_groups.h"

#include <math.h>

struct MpuMotion32 {
    CoprocStat_MpuQuaternion quat;
    CoprocStat_MpuVector accel;
    CoprocStat_MpuVector gyro;
};

enum MpuSendType : uint8_t {
    NEVER = 0,
    ONCE = 1,
    PERIODIC = 2,
};

static bool mpuInitialized = false;
static MpuSendType mpuSendIndicator = MpuSendType::NEVER;
static MpuMotion32 mpuAggrData;
static constexpr uint32_t mpuTickPeriodMs = 10;
static uint32_t mpuAggrCounter = 0;
static uint16_t compressCoef = 4;

void mpuReset() {
    if (mpuInitialized) {
        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
        mpu_setDMPEnabled(false);
        mpuInitialized = false;
        mpuSendIndicator = MpuSendType::NEVER;
    }
}

void mpuCreate() { mpu6050.devAddr = mpu_ADDRESS_AD0_HIGH; }

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
void mpuInitialize() {
    if (mpuInitialized) {
        return;
    }
    mpuInitialized = true;

    mpu_reset();
    vTaskDelay(pdMS_TO_TICKS(50));

    if (!mpu_testConnection()) {
        DEBUG("MPU6050 is not connected\n");
        CoprocStat status = {
            .which_payload = CoprocStat_faultStat_tag,
            .payload = { 
                .faultStat = {
                    .which_fault = CoprocStat_FaultStat_mpuFault_tag,
                },
            },
        };
        dispatcherEnqueueStatus(status);
        return;
    }

    mpu_setSleepEnabled(false);

    mpu_setClockSource(mpu_CLOCK_PLL_ZGYRO);
    I2Cdev_writeByte(mpu6050.devAddr, mpu_RA_INT_ENABLE,
        1 << mpu_INTERRUPT_FIFO_OFLOW_BIT | 1 << mpu_INTERRUPT_DMP_INT_BIT);
    mpu_setRate(4);
    mpu_setExternalFrameSync(mpu_EXT_SYNC_TEMP_OUT_L);
    mpu_setDLPFMode(mpu_DLPF_BW_42);
    mpu_setFullScaleGyroRange(mpu_GYRO_FS_2000);
    mpu_setFullScaleAccelRange(mpu_ACCEL_FS_2);

    mpu_writeMemoryBlock(dmpCode20, sizeof(dmpCode20));

    unsigned char dmpUpdate[] = { 0x00, 1 };
    mpu_writeMemoryBlock(dmpUpdate, 0x02, 0x02,
        0x16); // Lets write the dmpUpdate data to the Firmware image, we have 2 bytes to write in bank 0x02 with the Offset 0x16

    mpu_setDMPConfig1(0x03);
    mpu_setDMPConfig2(0x00);
    mpu_setOTPBankValid(false);

    mpu_setFIFOEnabled(true);

    mpu_setDMPEnabled(false);
    mpu_resetFIFO();
    mpu_resetDMP();

    mpu_setOTPBankValid(false);

    mpu_setDMPEnabled(true);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void mpuOnIntTriggered() { i2cSetEventFlagFromIsr(I2C_MPU_TICK); }

static void mpuCalculateYawPitchRoll(
    const CoprocStat_MpuQuaternion& q, CoprocStat_MpuVector& yawPitchRoll) {
    const float qw = (q.w / compressCoef) / 16384.f;
    const float qx = (q.x / compressCoef) / 16384.f;
    const float qy = (q.y / compressCoef) / 16384.f;
    const float qz = (q.z / compressCoef) / 16384.f;

    const float gx = 2 * (qx * qz - qw * qy);
    const float gy = 2 * (qw * qx + qy * qz);
    const float gz = qw * qw - qx * qx - qy * qy + qz * qz;

    const constexpr float PI = 3.14159f;

    float ex = atan2f(2 * qx * qy - 2 * qw * qz, 2 * qw * qw + 2 * qx * qx - 1)
        * (180.f / PI);
    float ey = atan2f(gx, sqrtf(gy * gy + gz * gz)) * (180.f / PI);
    float ez = atan2f(gy, gz) * (180.f / PI);

    if (gz < 0) {
        if (ey > 0) {
            ey = PI - ey;
        } else {
            ey = -PI - ey;
        }
    }

    yawPitchRoll.x = ex * 16384.f;
    yawPitchRoll.y = ey * 16384.f;
    yawPitchRoll.z = ez * 16384.f;
}

static void mpuSend(const MpuMotion32& data) {
    // DEBUG(
    //     "DEBUG MPU SEND [%d] acc: x:%d; y:%d; z:%d | gyro: x:%d; y:%d; z:%d\n",
    //     mpuAggrCounter, data.accel.x, data.accel.y, data.accel.z, data.gyro.x,
    //     data.gyro.y, data.gyro.z);

    CoprocStat status = {
        .which_payload = CoprocStat_mpuStat_tag,
    };

    auto& mpuStat = status.payload.mpuStat;

    memcpy(&mpuStat.accel, &mpuAggrData.accel, sizeof(CoprocStat_MpuVector));
    mpuStat.has_accel = true;

    memcpy(&mpuStat.gyro, &mpuAggrData.gyro, sizeof(CoprocStat_MpuVector));
    mpuStat.has_gyro = true;

    memcpy(&mpuStat.quat, &mpuAggrData.quat, sizeof(CoprocStat_MpuQuaternion));
    mpuStat.has_quat = true;

    mpuCalculateYawPitchRoll(mpuStat.quat, mpuStat.yawPitchRoll);
    mpuStat.has_yawPitchRoll = true;

    mpuStat.compressCoef = compressCoef;
    dispatcherEnqueueStatus(status);
}

void mpuTick() {
    uint16_t fifoCount = mpu_getFIFOCount();
    if (fifoCount >= 1024) {
        DEBUG("fifo overflow, resetting");
        mpu_resetFIFO();
        return;
    }

    std::array<uint8_t, 42> packet;

    while (fifoCount >= packet.size()) {
        int read_res = I2Cdev_readBytes(
            mpu6050.devAddr, mpu_RA_FIFO_R_W, packet.size(), packet.data(), 0);
        if (read_res < 0) {
            break;
        }

        const int16_t qw = (packet[0] << 8) | packet[1];
        const int16_t qx = (packet[4] << 8) | packet[5];
        const int16_t qy = (packet[8] << 8) | packet[9];
        const int16_t qz = (packet[12] << 8) | packet[13];

        const int16_t ax = (packet[28] << 8) | packet[29];
        const int16_t ay = (packet[32] << 8) | packet[33];
        const int16_t az = (packet[36] << 8) | packet[37];

        const int16_t gx = (packet[16] << 8) | packet[17];
        const int16_t gy = (packet[20] << 8) | packet[21];
        const int16_t gz = (packet[24] << 8) | packet[25];

        mpuAggrData.quat.w += qw;
        mpuAggrData.quat.x += qx;
        mpuAggrData.quat.y += qy;
        mpuAggrData.quat.z += qz;

        mpuAggrData.accel.x += ax;
        mpuAggrData.accel.y += ay;
        mpuAggrData.accel.z += az;

        mpuAggrData.gyro.x += gx;
        mpuAggrData.gyro.y += gy;
        mpuAggrData.gyro.z += gz;

        mpuAggrCounter++;
        if (mpuAggrCounter >= compressCoef) {
            if (mpuSendIndicator) {
                mpuSend(mpuAggrData);
                if (mpuSendIndicator == MpuSendType::ONCE) {
                    mpuSendIndicator = MpuSendType::NEVER;
                }
            }

            memset(&mpuAggrData, 0, sizeof(mpuAggrData));
            mpuAggrCounter = 0;
        }

        fifoCount -= read_res;
    }
}

static int16_t readMpuOffsetReg(uint8_t addr) {
    uint8_t data[2];
    I2Cdev_readBytes(mpu6050.devAddr, addr, 2, data, 0);
    return (((int16_t)data[0]) << 8) | data[1];
}

static void writeMpuOffsetReg(uint8_t addr, int16_t value) {
    uint8_t data[2] = { uint8_t(((uint16_t)value) >> 8),
        uint8_t(((uint16_t)value) & 0xFF) };
    I2Cdev_writeBytes(mpu6050.devAddr, addr, 2, data);
}

static void mpuCalibrationPid(uint8_t addr_read, uint8_t addr_write,
    uint16_t loops, float kP, float kI, bool is_accel,
    int16_t out_calibration_data[3]) {
    const uint8_t write_stride = 2;
    uint8_t bit_zero[3];
    float i_term[3];

    uint16_t gravity = 0;
    if (is_accel) {
        gravity = 16384 >> mpu_getFullScaleAccelRange();
    }

    const uint16_t target_error = is_accel ? 100 : 5;

    DEBUG("Starting MPU calibration\n");

    for (int i = 0; i < 3; i++) {
        int16_t data;
        data = readMpuOffsetReg(addr_write + (i * write_stride));

        // Capture Bit Zero to properly handle Accelerometer calibration
        if (is_accel) {
            bit_zero[i] = data & 1;
            i_term[i] = data * 8.f;
        } else {
            i_term[i] = data * 4.f;
        }
    }

    for (uint16_t loop_iter = 0; loop_iter < loops; ++loop_iter) {
        uint16_t readings_within_margin = 0;
        for (uint16_t sample_idx = 0; sample_idx < 0xFFFFF; ++sample_idx) {
            uint32_t readings_sum = 0;

            for (int i = 0; i < 3; i++) {
                int16_t reading_16 = 0;
                reading_16 = readMpuOffsetReg(addr_read + (i * 2));

                int32_t reading = reading_16;

                if (is_accel && i == 2) {
                    reading -= gravity;
                }

                readings_sum += abs(reading);

                float error = -reading;
                float p_term = kP * error;
                i_term[i] += (error * 0.001f) * kI;

                int16_t computed_offset;
                if (is_accel) {
                    computed_offset = round((p_term + i_term[i]) / 8);
                    computed_offset = (computed_offset & 0xFFFE) | bit_zero[i];
                } else {
                    computed_offset = round((p_term + i_term[i]) / 4);
                }

                //DEBUG("  %d: %d %d -> %d\n", i, reading, reading_16, computed_offset);

                writeMpuOffsetReg(
                    addr_write + (i * write_stride), computed_offset);
            }

            if (sample_idx % 100 == 99) {
                DEBUG("* %lu\n", readings_sum);
            }

            if (readings_sum < target_error) {
                ++readings_within_margin;
            }

            if (readings_sum < target_error && sample_idx > 10
                && readings_within_margin >= 10) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        DEBUG(".\n");
        kP *= .75f;
        kI *= .75f;

        for (int i = 0; i < 3; i++) {
            int16_t computed_offset;
            if (is_accel) {
                computed_offset = round(i_term[i] / 8);
                computed_offset = (computed_offset & 0xFFFE) | bit_zero[i];
            } else {
                computed_offset = round(i_term[i] / 4);
            }

            out_calibration_data[i] = computed_offset;
            writeMpuOffsetReg(addr_write + (i * write_stride), computed_offset);
        }
    }

    mpu_resetFIFO();
    mpu_resetDMP();
}

static void mpuCalibrateAccel(int16_t out_calibration_data[3]) {
    mpuCalibrationPid(mpu_RA_ACCEL_XOUT_H, mpu_RA_XA_OFFS_H, 5, 0.3f, 20.f,
        true, out_calibration_data);
}

static void mpuCalibrateGyro(int16_t out_calibration_data[3]) {
    mpuCalibrationPid(mpu_RA_GYRO_XOUT_H, mpu_RA_XG_OFFS_USRH, 5, 0.3f, 90.f,
        false, out_calibration_data);
}

static void mpuRestoreCalibrationData(const MpuCalibrationData& data) {
    const uint8_t write_stride = 2;

    auto data_cast = (const int16_t*)data.data;

    for (int i = 0; i < 3; i++) {
        writeMpuOffsetReg(mpu_RA_XA_OFFS_H + (i * write_stride), data_cast[i]);
    }

    for (int i = 0; i < 3; i++) {
        writeMpuOffsetReg(
            mpu_RA_XG_OFFS_USRH + (i * write_stride), data_cast[i + 3]);
    }

    mpu_resetFIFO();
    mpu_resetDMP();

    if (mpuInitialized) {
        mpu_setDMPEnabled(true);
    }
}

void mpuDispatch(const CoprocReq_MpuReq& req) {
    switch (req.which_mpuCmd) {
    case CoprocReq_MpuReq_init_tag:
        mpuInitialize();
        break;
    case CoprocReq_MpuReq_oneSend_tag: {
        if (mpuSendIndicator == MpuSendType::NEVER) {
            mpuSendIndicator = MpuSendType::ONCE;
        }
        break;
    }
    case CoprocReq_MpuReq_startSend_tag: {
        mpuSendIndicator = MpuSendType::PERIODIC;
        break;
    }
    case CoprocReq_MpuReq_stopSend_tag: {
        mpuSendIndicator = MpuSendType::NEVER;
        break;
    }
    case CoprocReq_MpuReq_setCompressCoef_tag: {
        uint16_t coef = req.mpuCmd.setCompressCoef;
        if (coef > 0 && coef <= 20) {
            compressCoef = coef;
            DEBUGLN("MPU6050 set new compression coef: %d", compressCoef);
        }
        break;
    }
    case CoprocReq_MpuReq_getCompressCoef_tag: {
        CoprocStat status = {
            .which_payload = CoprocStat_mpuStat_tag,
            .payload = {
                .mpuStat = {
                    .compressCoef = compressCoef,
                },
            },
        };
        dispatcherEnqueueStatus(status);
        break;
    }
    case CoprocReq_MpuReq_calibrateOffsets_tag: {
        DEBUG("Calibrating MPU offsets\n");
        int16_t calibration_data[6] = {};
        mpuCalibrateAccel(calibration_data);
        mpuCalibrateGyro(calibration_data + 3);

        CoprocStat status = {
            .which_payload = CoprocStat_mpuCalibrationDone_tag,
        };
        static_assert(sizeof(status.payload.mpuCalibrationDone.data)
            == sizeof(calibration_data));
        memcpy(status.payload.mpuCalibrationDone.data, calibration_data,
            sizeof(calibration_data));
        dispatcherEnqueueStatus(status);
        break;
    }
    case CoprocReq_MpuReq_restoreCalibrationData_tag: {
        mpuRestoreCalibrationData(req.mpuCmd.restoreCalibrationData);
        break;
    }
    };
}
