/**
 * @file SensorManager.cpp
 * @brief Implementation of centralized sensor management
 *
 * See SensorManager.h for design notes and usage.
 */

#include "SensorManager.h"
#include "PersistentStorage.h"
#include "../pins.h"
#include "../utility.h"
#include <Wire.h>

// ============================================================================
// STATIC MEMBER INITIALIZATION
// ============================================================================

IMUDriver    SensorManager::imu_;
FusionWrapper SensorManager::fusion_(IMU_UPDATE_FREQ_HZ, FUSION_GYRO_RECOVERY_DPS);

bool     SensorManager::imuInitialized_ = false;
uint32_t SensorManager::lastFusionMicros_ = 0;
bool     SensorManager::imuSamplePending_ = false;
uint16_t SensorManager::imuTimingAvgUs_ = 0;
uint16_t SensorManager::imuTimingPeakUs_ = 0;
uint16_t SensorManager::imuTimingMaxUs_ = 0;

UltrasonicDriver SensorManager::ultrasonics_[SENSOR_MAX_ULTRASONICS];
uint8_t          SensorManager::ultrasonicCount_   = 0;
bool             SensorManager::ultrasonicFound_[SENSOR_MAX_ULTRASONICS]   = {};
uint16_t         SensorManager::ultrasonicDistMm_[SENSOR_MAX_ULTRASONICS]  = {};

float   SensorManager::batteryVoltage_   = 0.0f;
float   SensorManager::rail5VVoltage_    = 0.0f;
float   SensorManager::servoVoltage_     = 0.0f;

MagCalData SensorManager::magCal_ = {
    MAG_CAL_STATE_IDLE, 0,
    0.0f, 0.0f,  0.0f, 0.0f,  0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    false
};
bool  SensorManager::magCalBackupValid_ = false;
float SensorManager::magCalBackupOffset_[3] = {0.0f, 0.0f, 0.0f};
float SensorManager::magCalBackupMatrix_[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f
};

bool     SensorManager::initialized_   = false;
uint32_t SensorManager::updateCount_   = 0;
uint32_t SensorManager::lastUpdateTime_ = 0;
uint16_t SensorManager::ultrasonicTimingAvgUs_ = 0;
uint16_t SensorManager::ultrasonicTimingPeakUs_ = 0;
uint16_t SensorManager::ultrasonicTimingMaxUs_ = 0;

// I2C addresses for configured sensors (from config.h)
static const uint8_t kUltrasonicAddrs[4] = {
    ULTRASONIC_0_I2C_ADDR, ULTRASONIC_1_I2C_ADDR,
    ULTRASONIC_2_I2C_ADDR, ULTRASONIC_3_I2C_ADDR
};

namespace {

void recordTiming(uint16_t elapsedUs,
                  uint16_t &avgUs,
                  uint16_t &peakUs,
                  uint16_t &maxUs) {
    if (avgUs == 0U) {
        avgUs = elapsedUs;
    } else {
        avgUs = (uint16_t)(((uint32_t)avgUs * 7U + (uint32_t)elapsedUs) >> 3);
    }
    if (elapsedUs > peakUs) {
        peakUs = elapsedUs;
    }
    if (elapsedUs > maxUs) {
        maxUs = elapsedUs;
    }
}

void setIdentityMatrix(float matrix[9]) {
    for (uint8_t i = 0; i < 9; i++) {
        matrix[i] = 0.0f;
    }
    matrix[0] = 1.0f;
    matrix[4] = 1.0f;
    matrix[8] = 1.0f;
}

} // namespace

// ============================================================================
// INITIALIZATION
// ============================================================================

void SensorManager::init() {
    if (initialized_) return;

    // Bring up the shared I2C bus before any sensor or servo driver touches it.
    // A wire timeout is essential here: without it, a sick device can hold
    // setup() or the sensor task forever and starve UART servicing.
    Wire.begin();
    Wire.setClock(I2C_INIT_CLOCK_HZ);
    Wire.setWireTimeout(I2C_WIRE_TIMEOUT_US, true);

    // ADC reference: use default (AVCC = 5 V)
    analogReference(DEFAULT);

    // Initial voltage readings
    updateVoltages();

    // --- IMU ---
#if IMU_ENABLED
    imuInitialized_ = imu_.init(IMU_AD0_VAL);

    if (imuInitialized_) {
        // Apply fusion settings from config.h
        fusion_.setSettings(FUSION_GAIN,
                            FUSION_ACCEL_REJECTION,
                            FUSION_MAG_REJECTION,
                            (float)FUSION_RECOVERY_PERIOD);

        // Load magnetometer calibration from persistent storage if available
        initMagCalFromStorage();

#ifdef DEBUG_SENSOR
        DEBUG_SERIAL.println(F("[SensorManager] IMU OK"));
        if (magCal_.savedToEeprom) {
            DEBUG_SERIAL.println(F("[SensorManager] Mag cal loaded from EEPROM (9-DOF)"));
        } else {
            DEBUG_SERIAL.println(F("[SensorManager] No mag cal found; running 6-DOF"));
        }
#endif
    } else {
#ifdef DEBUG_SENSOR
        DEBUG_SERIAL.println(F("[SensorManager] WARNING: IMU not detected"));
#endif
    }
#endif

    // --- Ultrasonic sensors ---
#if ULTRASONIC_COUNT > 0
    ultrasonicCount_ = 0;
    for (uint8_t i = 0; i < ULTRASONIC_COUNT && i < SENSOR_MAX_ULTRASONICS; i++) {
        ultrasonicFound_[i] = ultrasonics_[i].init(kUltrasonicAddrs[i]);
        if (ultrasonicFound_[i]) {
            ultrasonicCount_++;
        }
#ifdef DEBUG_SENSOR
        if (ultrasonicFound_[i]) {
            DEBUG_SERIAL.print(F("[SensorManager] Ultrasonic "));
            DEBUG_SERIAL.print(i);
            DEBUG_SERIAL.println(F(": OK"));
        } else {
            DEBUG_SERIAL.print(F("[SensorManager] WARNING: Ultrasonic "));
            DEBUG_SERIAL.print(i);
            DEBUG_SERIAL.println(F(" not detected (configured but missing)"));
        }
#endif
    }
#endif

    // Runtime polling restores the faster IMU-oriented bus speed on each tick,
    // but leave init() with that same runtime baseline to avoid surprising any
    // later startup-time users of the shared Wire bus.
    Wire.setClock(I2C_BUS_CLOCK_HZ);

    lastFusionMicros_ = micros();
    imuSamplePending_ = false;
    initialized_   = true;

#ifdef DEBUG_SENSOR
    DEBUG_SERIAL.print(F("[SensorManager] Battery: "));
    DEBUG_SERIAL.print(batteryVoltage_);
    DEBUG_SERIAL.println(F(" V"));
#endif
}

// ============================================================================
// tick — 100 Hz dispatcher (called from the soft scheduler)
// ============================================================================

void SensorManager::tick() {
    if (!initialized_) return;

    // Other shared-bus users such as the PCA9685 servo driver may temporarily
    // lower the Wire clock. Restore the sensor polling speed before every
    // sensor-dispatch tick so IMU reads do not expand enough to starve the
    // loop-owned DC compute round.
    Wire.setClock(I2C_BUS_CLOCK_HZ);

    static uint8_t counter = 0;

    lastUpdateTime_ = millis();
    updateCount_++;

    update100Hz();                         // every tick    (100 Hz)
    if ((counter & 1) == 0) update50Hz();  // even ticks   ( 50 Hz)
    if (counter == 0)        update10Hz(); // tick 0 only  ( 10 Hz)

    if (++counter >= 10) counter = 0;
}

void SensorManager::isrTick() {
    tick();
}

// ============================================================================
// update100Hz — alternating IMU/Fusion lane (100 Hz)
// ============================================================================

void SensorManager::update100Hz() {
    static bool imuReadPhase = true;

#if IMU_ENABLED
    if (imuReadPhase) {
        uint32_t imuStartUs = micros();
        if (imuInitialized_ && imu_.update(true)) {
            imuSamplePending_ = true;
            if (magCal_.state == MAG_CAL_STATE_SAMPLING) {
                updateMagCalSampling();
            }
        } else {
            imuSamplePending_ = false;
        }
        recordTiming(Utility::clampElapsedUs(micros() - imuStartUs),
                     imuTimingAvgUs_,
                     imuTimingPeakUs_,
                     imuTimingMaxUs_);
    } else {
        uint32_t imuStartUs = micros();
        if (imuInitialized_ && imuSamplePending_) {
            uint32_t now = micros();
            float dtSec = (lastFusionMicros_ == 0U)
                              ? (1.0f / IMU_UPDATE_FREQ_HZ)
                              : ((now - lastFusionMicros_) * 1e-6f);
            lastFusionMicros_ = now;

            if (dtSec <= 0.0f || dtSec > 0.1f) {
                dtSec = 1.0f / IMU_UPDATE_FREQ_HZ;
            }

            float ax = imu_.getAccX() * 0.001f;
            float ay = imu_.getAccY() * 0.001f;
            float az = imu_.getAccZ() * 0.001f;

            if (isMagCalibrated()) {
                fusion_.update(
                    imu_.getGyrX(), imu_.getGyrY(), imu_.getGyrZ(),
                    ax, ay, az,
                    imu_.getMagX(), imu_.getMagY(), imu_.getMagZ(),
                    dtSec
                );
            } else {
                fusion_.updateNoMag(
                    imu_.getGyrX(), imu_.getGyrY(), imu_.getGyrZ(),
                    ax, ay, az,
                    dtSec
                );
            }
            imuSamplePending_ = false;
        }
        recordTiming(Utility::clampElapsedUs(micros() - imuStartUs),
                     imuTimingAvgUs_,
                     imuTimingPeakUs_,
                     imuTimingMaxUs_);
    }
#endif

    if (!imuReadPhase) {
        static uint8_t ultrasonicDivider = 0;
        uint32_t ultrasonicStartUs = micros();
        if (++ultrasonicDivider >= 5U) {
            ultrasonicDivider = 0U;
            for (uint8_t i = 0; i < ULTRASONIC_COUNT && i < SENSOR_MAX_ULTRASONICS; i++) {
                if (ultrasonicFound_[i]) {
                    ultrasonicDistMm_[i] = ultrasonics_[i].getDistanceMm();
                }
            }
        }
        recordTiming(Utility::clampElapsedUs(micros() - ultrasonicStartUs),
                     ultrasonicTimingAvgUs_,
                     ultrasonicTimingPeakUs_,
                     ultrasonicTimingMaxUs_);
    }

    imuReadPhase = !imuReadPhase;
}

void SensorManager::update50Hz() {
    // Reserved medium-rate lane. The control-friendly profile alternates IMU
    // bus reads and Fusion work above so a single sensor tick does not block
    // loop() long enough to miss a DC control round.
}

// ============================================================================
// update10Hz — Voltages (10 Hz)
// ============================================================================

void SensorManager::update10Hz() {
    updateVoltages();
}

// ============================================================================
// IMU / FUSION OUTPUT
// ============================================================================

void SensorManager::getQuaternion(float& w, float& x, float& y, float& z) {
    fusion_.getQuaternion(w, x, y, z);
}

void SensorManager::getEarthAcceleration(float& x, float& y, float& z) {
    fusion_.getEarthAcceleration(x, y, z);
}

int16_t SensorManager::getRawAccX() { return imu_.getRawAccX(); }
int16_t SensorManager::getRawAccY() { return imu_.getRawAccY(); }
int16_t SensorManager::getRawAccZ() { return imu_.getRawAccZ(); }
int16_t SensorManager::getRawGyrX() { return imu_.getRawGyrX(); }
int16_t SensorManager::getRawGyrY() { return imu_.getRawGyrY(); }
int16_t SensorManager::getRawGyrZ() { return imu_.getRawGyrZ(); }
int16_t SensorManager::getRawMagX() { return imu_.getRawMagX(); }
int16_t SensorManager::getRawMagY() { return imu_.getRawMagY(); }
int16_t SensorManager::getRawMagZ() { return imu_.getRawMagZ(); }
int16_t SensorManager::getRawTempDeciC() { return (int16_t)(imu_.getTemp() * 10.0f); }

// ============================================================================
// RANGE SENSOR OUTPUT
// ============================================================================

bool SensorManager::isUltrasonicFound(uint8_t idx) {
    if (idx >= SENSOR_MAX_ULTRASONICS) return false;
    return ultrasonicFound_[idx];
}

uint8_t SensorManager::getUltrasonicConfiguredCount() { return ULTRASONIC_COUNT; }

uint16_t SensorManager::getUltrasonicDistanceMm(uint8_t idx) {
    if (idx >= SENSOR_MAX_ULTRASONICS || !ultrasonicFound_[idx]) return 0;
    return ultrasonicDistMm_[idx];
}

void SensorManager::clearTimingPeaks() {
    imuTimingPeakUs_ = 0;
    ultrasonicTimingPeakUs_ = 0;
}

// ============================================================================
// VOLTAGE MONITORING
// ============================================================================

float SensorManager::getBatteryVoltage() { return batteryVoltage_; }
float SensorManager::get5VRailVoltage()  { return rail5VVoltage_;  }
float SensorManager::getServoVoltage()   { return servoVoltage_;   }

bool SensorManager::isBatteryLow(float threshold) {
    return (batteryVoltage_ > 0.0f && batteryVoltage_ < threshold);
}

bool SensorManager::isBatteryCritical() {
    return (batteryVoltage_ > 0.0f && batteryVoltage_ < VBAT_CUTOFF_V);
}

bool SensorManager::isBatteryOvervoltage() {
    return (batteryVoltage_ > VBAT_OVERVOLTAGE_V);
}

// ============================================================================
// MAGNETOMETER CALIBRATION
// ============================================================================

void SensorManager::startMagCalibration() {
    if (isMagCalibrated()) {
        imu_.getMagCalibration(
            magCalBackupOffset_[0], magCalBackupOffset_[1], magCalBackupOffset_[2], magCalBackupMatrix_);
        magCalBackupValid_ = true;
    } else {
        magCalBackupValid_ = false;
        magCalBackupOffset_[0] = 0.0f;
        magCalBackupOffset_[1] = 0.0f;
        magCalBackupOffset_[2] = 0.0f;
        setIdentityMatrix(magCalBackupMatrix_);
    }

    magCal_.state       = MAG_CAL_STATE_SAMPLING;
    magCal_.sampleCount = 0;
    magCal_.minX = magCal_.maxX = 0.0f;
    magCal_.minY = magCal_.maxY = 0.0f;
    magCal_.minZ = magCal_.maxZ = 0.0f;
    magCal_.offsetX = magCal_.offsetY = magCal_.offsetZ = 0.0f;
    magCal_.savedToEeprom = false;

    // Clear the active calibration during sampling so the bridge receives raw
    // magnetometer values in the accel/gyro frame.
    imu_.clearMagCalibration();

    // Revert to 6-DOF during calibration (mag data is unreliable without offsets)
    fusion_.reset();

#ifdef DEBUG_SENSOR
    DEBUG_SERIAL.println(F("[MagCal] Calibration started — rotate robot through all orientations"));
#endif
}

void SensorManager::cancelMagCalibration() {
    restoreMagCalibrationBackup();
#ifdef DEBUG_SENSOR
    DEBUG_SERIAL.println(F("[MagCal] Calibration cancelled"));
#endif
}

bool SensorManager::saveMagCalibration() {
    if (magCal_.sampleCount < MAG_CAL_MIN_SAMPLES) {
#ifdef DEBUG_SENSOR
        DEBUG_SERIAL.print(F("[MagCal] Insufficient samples: "));
        DEBUG_SERIAL.print(magCal_.sampleCount);
        DEBUG_SERIAL.print(F(" / "));
        DEBUG_SERIAL.println(MAG_CAL_MIN_SAMPLES);
#endif
        magCal_.state = MAG_CAL_STATE_ERROR;
        return false;
    }

    float identity[9];
    setIdentityMatrix(identity);
    applyMagCalibration(magCal_.offsetX, magCal_.offsetY, magCal_.offsetZ, identity);
    return true;
}

void SensorManager::applyMagCalibration(float ox, float oy, float oz, const float matrix[9]) {
    magCal_.offsetX = ox;
    magCal_.offsetY = oy;
    magCal_.offsetZ = oz;

    imu_.setMagCalibration(ox, oy, oz, matrix);

    PersistentStorage::setMagCalibration(ox, oy, oz, matrix);

    magCal_.state         = MAG_CAL_STATE_SAVED;
    magCal_.savedToEeprom = PersistentStorage::hasMagCalibration();
    magCalBackupValid_    = false;

    fusion_.reset();

#ifdef DEBUG_SENSOR
    DEBUG_SERIAL.println(F("[MagCal] Full calibration saved and 9-DOF mode activated"));
    DEBUG_SERIAL.print(F("  offsets (uT): "));
    DEBUG_SERIAL.print(ox); DEBUG_SERIAL.print(F(", "));
    DEBUG_SERIAL.print(oy); DEBUG_SERIAL.print(F(", "));
    DEBUG_SERIAL.println(oz);
#endif
}

void SensorManager::clearMagCalibration() {
    PersistentStorage::clearMagCalibration();

    imu_.clearMagCalibration();
    magCal_.savedToEeprom = false;
    magCal_.state         = MAG_CAL_STATE_IDLE;
    magCal_.sampleCount   = 0;
    magCalBackupValid_    = false;

    fusion_.reset();

#ifdef DEBUG_SENSOR
    DEBUG_SERIAL.println(F("[MagCal] Calibration cleared; reverted to 6-DOF"));
#endif
}

// ============================================================================
// INTERNAL: LOAD FROM PERSISTENT STORAGE
// ============================================================================

void SensorManager::initMagCalFromStorage() {
    float ox, oy, oz;
    float matrix[9];
    if (!PersistentStorage::getMagCalibration(ox, oy, oz, matrix)) {
        magCal_.state = MAG_CAL_STATE_IDLE;
        magCal_.savedToEeprom = false;
        return;
    }

    magCal_.offsetX       = ox;
    magCal_.offsetY       = oy;
    magCal_.offsetZ       = oz;
    magCal_.savedToEeprom = true;
    magCal_.state         = MAG_CAL_STATE_SAVED;

    imu_.setMagCalibration(ox, oy, oz, matrix);
}

void SensorManager::restoreMagCalibrationBackup() {
    magCal_.sampleCount = 0;

    if (magCalBackupValid_) {
        imu_.setMagCalibration(
            magCalBackupOffset_[0], magCalBackupOffset_[1], magCalBackupOffset_[2], magCalBackupMatrix_);
        magCal_.offsetX = magCalBackupOffset_[0];
        magCal_.offsetY = magCalBackupOffset_[1];
        magCal_.offsetZ = magCalBackupOffset_[2];
        magCal_.savedToEeprom = true;
        magCal_.state = MAG_CAL_STATE_SAVED;
    } else {
        imu_.clearMagCalibration();
        magCal_.offsetX = 0.0f;
        magCal_.offsetY = 0.0f;
        magCal_.offsetZ = 0.0f;
        magCal_.savedToEeprom = false;
        magCal_.state = MAG_CAL_STATE_IDLE;
    }

    magCalBackupValid_ = false;
    fusion_.reset();
}

// ============================================================================
// INTERNAL: MAG CAL SAMPLING (called every IMU update while SAMPLING)
// ============================================================================

void SensorManager::updateMagCalSampling() {
    // Offsets were zeroed by startMagCalibration(), so getMagX/Y/Z() returns raw µT.
    float mx = imu_.getMagX();
    float my = imu_.getMagY();
    float mz = imu_.getMagZ();

    if (magCal_.sampleCount == 0) {
        magCal_.minX = magCal_.maxX = mx;
        magCal_.minY = magCal_.maxY = my;
        magCal_.minZ = magCal_.maxZ = mz;
    } else {
        if (mx < magCal_.minX) magCal_.minX = mx;
        if (mx > magCal_.maxX) magCal_.maxX = mx;
        if (my < magCal_.minY) magCal_.minY = my;
        if (my > magCal_.maxY) magCal_.maxY = my;
        if (mz < magCal_.minZ) magCal_.minZ = mz;
        if (mz > magCal_.maxZ) magCal_.maxZ = mz;
    }

    magCal_.sampleCount++;

    // Compute running offsets (hard-iron = center of the ellipsoid)
    magCal_.offsetX = (magCal_.maxX + magCal_.minX) * 0.5f;
    magCal_.offsetY = (magCal_.maxY + magCal_.minY) * 0.5f;
    magCal_.offsetZ = (magCal_.maxZ + magCal_.minZ) * 0.5f;
}

// ============================================================================
// INTERNAL: VOLTAGE MONITORING
// ============================================================================

void SensorManager::updateVoltages() {
    // Battery (1:6 divider → multiply ADC voltage by 6)
    batteryVoltage_ = adcToVoltage(readADCAverage(PIN_VBAT_SENSE), 1.0f / 6.0f);

    // 5V rail (1:2 divider → multiply by 2)
    rail5VVoltage_  = adcToVoltage(readADCAverage(PIN_V5_SENSE), 1.0f / 2.0f);

    // Servo rail (1:3 divider → multiply by 3)
    servoVoltage_   = adcToVoltage(readADCAverage(PIN_VSERVO_SENSE), 1.0f / 3.0f);
}

uint16_t SensorManager::readADCAverage(uint8_t pin, uint8_t numSamples) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < numSamples; i++) {
        sum += analogRead(pin);
    }
    return (uint16_t)(sum / numSamples);
}

float SensorManager::adcToVoltage(uint16_t adcValue, float dividerRatio) {
    // ADC input voltage
    float vADC = (adcValue / 1023.0f) * ADC_VREF;
    // Actual voltage before the divider (dividerRatio = Vout / Vin, so Vin = Vout / ratio)
    return vADC / dividerRatio;
}
