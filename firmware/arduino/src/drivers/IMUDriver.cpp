/**
 * @file IMUDriver.cpp
 * @brief Implementation of ICM-20948 IMU driver (SparkFun library, no DMP)
 *
 * See IMUDriver.h for usage and design notes.
 */

#include "IMUDriver.h"

#if IMU_ENABLED
#include "../lib/SparkFun_9DoF_IMU_Breakout_-_ICM_20948_-_Arduino_Library/src/util/ICM_20948_REGISTERS.h"
#endif

namespace {

#if IMU_ENABLED

ICM_20948_ACCEL_CONFIG_FS_SEL_e accelRangeToEnum() {
    switch (IMU_ACCEL_RANGE_G) {
        case 2:  return gpm2;
        case 4:  return gpm4;
        case 8:  return gpm8;
        case 16: return gpm16;
        default: return gpm2;
    }
}

ICM_20948_GYRO_CONFIG_1_FS_SEL_e gyroRangeToEnum() {
    switch (IMU_GYRO_RANGE_DPS) {
        case 250:  return dps250;
        case 500:  return dps500;
        case 1000: return dps1000;
        case 2000: return dps2000;
        default:   return dps250;
    }
}

float accelLsbPerG() {
    switch (IMU_ACCEL_RANGE_G) {
        case 2:  return 16384.0f;
        case 4:  return 8192.0f;
        case 8:  return 4096.0f;
        case 16: return 2048.0f;
        default: return 16384.0f;
    }
}

float gyroLsbPerDps() {
    switch (IMU_GYRO_RANGE_DPS) {
        case 250:  return 131.0f;
        case 500:  return 65.5f;
        case 1000: return 32.8f;
        case 2000: return 16.4f;
        default:   return 131.0f;
    }
}

#endif

int16_t readBe16(const uint8_t *buf) {
    return (int16_t)((((uint16_t)buf[0]) << 8) | (uint16_t)buf[1]);
}

int16_t readLe16(const uint8_t *buf) {
    return (int16_t)((((uint16_t)buf[1]) << 8) | (uint16_t)buf[0]);
}

#if IMU_ENABLED

float rawAccelToMg(int16_t raw) {
    return (((float)raw) * 1000.0f) / accelLsbPerG();
}

float rawGyroToDps(int16_t raw) {
    return ((float)raw) / gyroLsbPerDps();
}

float rawTempToC(int16_t raw) {
    return ((((float)raw) - 21.0f) / 333.87f) + 21.0f;
}

float rawMagToUt(int16_t raw) {
    return ((float)raw) * 0.15f;
}

void remapMagToImuFrame(float &mx, float &my, float &mz) {
    // The ICM-20948 packages the accel/gyro die and the AK09916 magnetometer
    // die in different physical orientations. We treat accel/gyro as already
    // aligned with the rover body frame, so the magnetometer must be remapped
    // into that same frame before calibration offsets or Fusion AHRS use it.
    //
    // Fixed package relationship for ICM-20948:
    //   mag_X_body =  mag_X_ak09916
    //   mag_Y_body = -mag_Y_ak09916
    //   mag_Z_body = -mag_Z_ak09916
    my = -my;
    mz = -mz;
}

#endif

void setIdentityMatrix(float matrix[9]) {
    for (uint8_t i = 0; i < 9; i++) {
        matrix[i] = 0.0f;
    }
    matrix[0] = 1.0f;
    matrix[4] = 1.0f;
    matrix[8] = 1.0f;
}

void copyMatrix(float dst[9], const float src[9]) {
    for (uint8_t i = 0; i < 9; i++) {
        dst[i] = src[i];
    }
}

#if IMU_ENABLED

void applyMatrix3x3(const float matrix[9], float &x, float &y, float &z) {
    const float inX = x;
    const float inY = y;
    const float inZ = z;
    x = matrix[0] * inX + matrix[1] * inY + matrix[2] * inZ;
    y = matrix[3] * inX + matrix[4] * inY + matrix[5] * inZ;
    z = matrix[6] * inX + matrix[7] * inY + matrix[8] * inZ;
}

#endif

} // namespace

// ============================================================================
// CONSTRUCTOR
// ============================================================================

IMUDriver::IMUDriver()
    : initialized_(false)
    , accX_(0.0f), accY_(0.0f), accZ_(0.0f)
    , gyrX_(0.0f), gyrY_(0.0f), gyrZ_(0.0f)
    , magX_(0.0f), magY_(0.0f), magZ_(0.0f)
    , temp_(0.0f)
    , magOffX_(0.0f), magOffY_(0.0f), magOffZ_(0.0f)
{
    setIdentityMatrix(magMatrix_);
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool IMUDriver::init(uint8_t ad0Val) {
#if IMU_ENABLED
    // The shared I2C bus is configured once by SensorManager::init().
    // Do not change the global bus speed here; the same Wire bus is also used
    // by the ultrasonic sensors and PCA9685 servo controller.

    // Try to initialize the ICM-20948. The library will retry on failure.
    imu_.begin(Wire, ad0Val);

    if (imu_.status != ICM_20948_Stat_Ok) {
#ifdef DEBUG_SENSOR
        DEBUG_SERIAL.print(F("[IMU] Init failed: "));
        DEBUG_SERIAL.println(imu_.statusString());
#endif
        return false;
    }

    ICM_20948_fss_t fullScale = {};
    fullScale.a = accelRangeToEnum();
    fullScale.g = gyroRangeToEnum();
    if (imu_.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fullScale) != ICM_20948_Stat_Ok) {
#ifdef DEBUG_SENSOR
        DEBUG_SERIAL.print(F("[IMU] setFullScale failed: "));
        DEBUG_SERIAL.println(imu_.statusString());
#endif
        return false;
    }

    initialized_ = true;

#ifdef DEBUG_SENSOR
    DEBUG_SERIAL.print(F("[IMU] ICM-20948 initialized (AD0="));
    DEBUG_SERIAL.print(ad0Val);
    DEBUG_SERIAL.print(F(", addr=0x"));
    DEBUG_SERIAL.print(ad0Val ? 0x69 : 0x68, HEX);
    DEBUG_SERIAL.print(F(", accel=+/-"));
    DEBUG_SERIAL.print(IMU_ACCEL_RANGE_G);
    DEBUG_SERIAL.print(F("g, gyro=+/-"));
    DEBUG_SERIAL.print(IMU_GYRO_RANGE_DPS);
    DEBUG_SERIAL.println(F(" dps)"));
#endif

    return true;
#else
    (void)ad0Val;
    return false;
#endif
}

// ============================================================================
// DATA ACQUISITION
// ============================================================================

bool IMUDriver::dataReady() {
#if IMU_ENABLED
    if (!initialized_) return false;
    return imu_.dataReady();
#else
    return false;
#endif
}

bool IMUDriver::update(bool includeMag) {
#if IMU_ENABLED
    if (!initialized_) return false;

    // Direct register bursts must start from bank 0. The previous dataReady()
    // call happened to leave the device there; once that extra I2C poll was
    // removed, we need to select the bank explicitly again.
    if (imu_.setBank(0) != ICM_20948_Stat_Ok) {
        return false;
    }

    const uint8_t rawLen = includeMag ? 23U : 14U;
    uint8_t raw[23] = {};
    if (imu_.read(AGB0_REG_ACCEL_XOUT_H, raw, rawLen) != ICM_20948_Stat_Ok) {
        return false;
    }

    accX_ = rawAccelToMg(readBe16(&raw[0]));
    accY_ = rawAccelToMg(readBe16(&raw[2]));
    accZ_ = rawAccelToMg(readBe16(&raw[4]));

    gyrX_ = rawGyroToDps(readBe16(&raw[6]));
    gyrY_ = rawGyroToDps(readBe16(&raw[8]));
    gyrZ_ = rawGyroToDps(readBe16(&raw[10]));

    temp_ = rawTempToC(readBe16(&raw[12]));

    if (includeMag) {
        // startupDefault() configures the embedded AK09916 for continuous
        // 100 Hz sampling and maps ST1 + XYZ + ST2 into the external sensor
        // data registers right after TEMP_OUT. Reading the whole 23-byte block
        // directly is much cheaper than the library's getAGMT() helper, which
        // also re-reads scale configuration on every sample.
        const uint8_t magStatus1 = raw[14];
        const uint8_t magStatus2 = raw[22];
        const bool magReady = (magStatus1 & 0x01U) != 0U;
        const bool magOverflow = (magStatus2 & 0x08U) != 0U;

        if (magReady && !magOverflow) {
            float magX = rawMagToUt(readLe16(&raw[15]));
            float magY = rawMagToUt(readLe16(&raw[17]));
            float magZ = rawMagToUt(readLe16(&raw[19]));
            remapMagToImuFrame(magX, magY, magZ);

            magX -= magOffX_;
            magY -= magOffY_;
            magZ -= magOffZ_;
            applyMatrix3x3(magMatrix_, magX, magY, magZ);

            magX_ = magX;
            magY_ = magY;
            magZ_ = magZ;
        }
    }

    return true;
#else
    (void)includeMag;
    return false;
#endif
}

// ============================================================================
// MAGNETOMETER CALIBRATION
// ============================================================================

void IMUDriver::setMagCalibration(float ox, float oy, float oz, const float matrix[9]) {
    magOffX_ = ox;
    magOffY_ = oy;
    magOffZ_ = oz;
    copyMatrix(magMatrix_, matrix);
}

void IMUDriver::clearMagCalibration() {
    magOffX_ = 0.0f;
    magOffY_ = 0.0f;
    magOffZ_ = 0.0f;
    setIdentityMatrix(magMatrix_);
}

void IMUDriver::getMagCalibration(float& ox, float& oy, float& oz, float matrix[9]) const {
    ox = magOffX_;
    oy = magOffY_;
    oz = magOffZ_;
    copyMatrix(matrix, magMatrix_);
}

void IMUDriver::setMagOffset(float ox, float oy, float oz) {
    float identity[9];
    setIdentityMatrix(identity);
    setMagCalibration(ox, oy, oz, identity);
}

void IMUDriver::getMagOffset(float& ox, float& oy, float& oz) const {
    ox = magOffX_;
    oy = magOffY_;
    oz = magOffZ_;
}
