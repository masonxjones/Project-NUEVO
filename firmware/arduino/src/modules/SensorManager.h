/**
 * @file SensorManager.h
 * @brief Centralised sensor management with soft-scheduler dispatch
 *
 * SensorManager is driven by a 100 Hz soft task in the main loop. Each tick
 * dispatches work at three rates using an internal 0–9 counter:
 *
 *   update100Hz()  — every tick       → 100 Hz: alternating IMU read and
 *                                               Fusion/ultrasonic phases
 *   update50Hz()   — even ticks       →  50 Hz: reserved medium-rate lane
 *   update10Hz()   — tick 0 only      →  10 Hz: Voltages
 *
 * Call SensorManager::tick() from the soft scheduler. A compatibility wrapper
 * named isrTick() remains so older code continues to compile, but the bring-up
 * profile does not invoke sensor work from an ISR.
 *
 * ── How to add a new sensor ──────────────────────────────────────────────
 *  1. Initialise the driver in init().
 *  2. Add a member pointer/instance in the private section.
 *  3. Place the read call in update100Hz(), update50Hz(), or update10Hz()
 *     according to the sensor's required update rate.
 *  4. Expose the result via a getter (see getUltrasonicDistanceMm() as a model).
 * ─────────────────────────────────────────────────────────────────────────
 *
 * Magnetometer Calibration State Machine
 * ────────────────────────────────────────
 * Calibration is only allowed in IDLE firmware state. The Bridge sends
 * SENSOR_MAG_CAL_CMD; the result is streamed back via SENSOR_MAG_CAL_STATUS.
 *
 *   IDLE → startMagCalibration() → SAMPLING (collecting host-side fit samples)
 *        → applyMagCalibration() → SAVED (offset + matrix written to EEPROM, 9-DOF active)
 *        → cancelMagCalibration()→ restore previous calibration (or uncalibrated IDLE)
 *
 * Voltage Dividers (from PCB hardware design):
 *   VBAT  (A0): 50kΩ/10kΩ → 1:6 → 0–24 V maps to 0–4 V ADC
 *   V5    (A1): 10kΩ/10kΩ → 1:2 → 0–10 V maps to 0–5 V ADC
 *   VSERVO(A2): 20kΩ/10kΩ → 1:3 → 0–15 V maps to 0–5 V ADC
 */

#ifndef SENSORMANAGER_H
#define SENSORMANAGER_H

#include <Arduino.h>
#include <stdint.h>
#include "../config.h"
#include "../drivers/IMUDriver.h"
#include "../drivers/UltrasonicDriver.h"
#include "../lib/Fusion/FusionWrapper.h"
#include "../messages/TLV_Payloads.h"
#include "PersistentStorage.h"

// Maximum number of range sensors of each type (must match config.h defines)
#define SENSOR_MAX_ULTRASONICS  4

struct MagCalData {
    MagCalState state;
    uint16_t    sampleCount;
    float       minX, maxX;    // µT
    float       minY, maxY;
    float       minZ, maxZ;
    float       offsetX, offsetY, offsetZ;  // (max+min)/2 per axis
    bool        savedToEeprom;
};

// ============================================================================
// SENSOR MANAGER (Static class)
// ============================================================================

class SensorManager {
public:
    /**
     * @brief Initialize all enabled sensors
     *
     * Initializes I2C, IMU, ultrasonic sensor(s), and reads initial voltages.
     * Loads magnetometer calibration from EEPROM if available.
     * Call once in setup() before using any sensors.
     */
    static void init();

    /**
     * @brief Soft-scheduler dispatch entry point (100 Hz)
     *
     * Dispatches sensor reads at three rates using an internal 0–9 counter:
     *   - update100Hz()  every call       (100 Hz) — alternating IMU read and
     *                                                 Fusion/ultrasonic phases
     *   - update50Hz()   every 2nd call   ( 50 Hz) — reserved medium-rate lane
     *   - update10Hz()   every 10th call  ( 10 Hz) — Voltages
     *
     * Call from the soft scheduler, not from an ISR.
     */
    static void tick();

    /**
     * @brief Compatibility wrapper for older call sites.
     *
     * Equivalent to tick(). Present only to avoid breaking old references.
     */
    static void isrTick();

    // ========================================================================
    // IMU / FUSION OUTPUT
    // ========================================================================

    /**
     * @brief Get the latest orientation quaternion from Fusion AHRS
     *
     * In 9-DOF mode (magnetometer calibrated): yaw is absolute (north-referenced).
     * In 6-DOF fallback: yaw drifts slowly with gyro integration.
     */
    static void getQuaternion(float& w, float& x, float& y, float& z);

    /**
     * @brief Get linear acceleration in the earth (global/NWU) frame
     *
     * Gravity removed. A stationary sensor returns approximately (0, 0, 0).
     * Units: g (1 g = 9.81 m/s²).
     */
    static void getEarthAcceleration(float& x, float& y, float& z);

    /**
     * @brief Check if the IMU is connected and producing data
     */
    static bool isIMUAvailable() { return imuInitialized_; }

    /**
     * @brief Check if the magnetometer calibration is active (9-DOF fusion)
     *
     * This reflects the active runtime state, not whether the calibration was
     * successfully persisted to EEPROM. Persistence is reported separately via
     * MagCalData::savedToEeprom.
     */
    static bool isMagCalibrated() { return magCal_.state == MAG_CAL_STATE_SAVED; }

    /**
     * @brief Get latest raw IMU values (for TLV packing)
     */
    static int16_t getRawAccX();
    static int16_t getRawAccY();
    static int16_t getRawAccZ();
    static int16_t getRawGyrX();
    static int16_t getRawGyrY();
    static int16_t getRawGyrZ();
    static int16_t getRawMagX();
    static int16_t getRawMagY();
    static int16_t getRawMagZ();
    static int16_t getRawTempDeciC();

    // ========================================================================
    // RANGE SENSORS
    // ========================================================================

    /**
     * @brief Get number of ultrasonic sensors that responded during init
     */
    static uint8_t getUltrasonicCount() { return ultrasonicCount_; }

    /**
     * @brief True if the sensor at this slot responded during init.
     *
     * A sensor may be configured but absent on the I2C bus.
     * Use this to distinguish "configured but missing" from "not configured".
     */
    static bool isUltrasonicFound(uint8_t idx);

    /**
     * @brief Number of sensor slots enabled at compile time (from config.h).
     *
     * Compare with getUltrasonicCount() to detect missing sensors:
     *   missing = getUltrasonicConfiguredCount() - getUltrasonicCount()
     */
    static uint8_t getUltrasonicConfiguredCount();

    /**
     * @brief Get latest ultrasonic distance reading
     *
     * @param idx Slot index (0-based). Returns 0 if slot not found or out of range.
     */
    static uint16_t getUltrasonicDistanceMm(uint8_t idx);

    /**
     * @brief Range-sensor timing stats for debug/status reporting.
     *
     * These timings measure only the ultrasonic read loop, not the full
     * 100 Hz sensor task wrapper.
     */
    static uint16_t getUltrasonicTimingAvgUs() { return ultrasonicTimingAvgUs_; }
    static uint16_t getUltrasonicTimingPeakUs() { return ultrasonicTimingPeakUs_; }
    static uint16_t getUltrasonicTimingMaxUs() { return ultrasonicTimingMaxUs_; }
    static uint16_t getImuTimingAvgUs() { return imuTimingAvgUs_; }
    static uint16_t getImuTimingPeakUs() { return imuTimingPeakUs_; }
    static uint16_t getImuTimingMaxUs() { return imuTimingMaxUs_; }
    static void clearTimingPeaks();

    // ========================================================================
    // VOLTAGE MONITORING
    // ========================================================================

    static float getBatteryVoltage();
    static float get5VRailVoltage();
    static float getServoVoltage();
    static bool  isServoRailPresent() { return servoVoltage_ >= VSERVO_MIN_PRESENT_V; }

    /**
     * @brief Battery presence check
     *
     * Returns true when the measured battery voltage is at or above VBAT_MIN_PRESENT_V.
     * Returns false when no battery is connected (ADC reads ~0 V) or the voltage is
     * too low to safely run motors.  All actuator enable commands are silently rejected
     * when this returns false — the system does NOT enter ERROR state.
     */
    static bool  isBatteryPresent() { return batteryVoltage_ >= VBAT_MIN_PRESENT_V; }

    /**
     * @brief Battery under-voltage checks
     *
     * isBatteryLow()      — below VBAT_WARN_V    → warning / UI status only
     * isBatteryCritical() — below VBAT_CUTOFF_V  → hard safety (motors disabled)
     */
    static bool  isBatteryLow(float threshold = VBAT_LOW_THRESHOLD);
    static bool  isBatteryCritical();

    /**
     * @brief Battery over-voltage check
     *
     * Returns true when battery is above VBAT_OVERVOLTAGE_V.
     * Typically indicates wrong charger, wrong chemistry, or measurement error.
     */
    static bool  isBatteryOvervoltage();

    // ========================================================================
    // MAGNETOMETER CALIBRATION API
    // ========================================================================

    /**
     * @brief Start magnetometer calibration sampling
     *
     * Resets min/max trackers and begins collecting samples.
     * Only accepted when firmware is in IDLE state (enforced by caller).
     */
    static void startMagCalibration();

    /**
     * @brief Stop calibration sampling without saving
     */
    static void cancelMagCalibration();

    /**
     * @brief Save current computed calibration offsets to EEPROM and activate
     *
     * Requires MAG_CAL_MIN_SAMPLES samples. Returns false if insufficient.
     *
     * @return True if offsets were saved successfully
     */
    static bool saveMagCalibration();

    /**
     * @brief Apply user-provided hard/soft-iron calibration directly and save to EEPROM
     *
     * Skips the sampling phase; offset + matrix are provided by the Bridge.
     */
    static void applyMagCalibration(float ox, float oy, float oz, const float matrix[9]);

    /**
     * @brief Clear EEPROM calibration and revert to 6-DOF mode
     */
    static void clearMagCalibration();

    /**
     * @brief Read current calibration state (for SENSOR_MAG_CAL_STATUS TLV)
     */
    static const MagCalData& getMagCalData() { return magCal_; }

    // ========================================================================
    // STATISTICS
    // ========================================================================

    static uint32_t getUpdateCount()    { return updateCount_; }
    static uint32_t getLastUpdateTime() { return lastUpdateTime_; }

private:
    // ---- IMU + Fusion ----
    static IMUDriver    imu_;
    static FusionWrapper fusion_;
    static bool imuInitialized_;
    static uint32_t lastFusionMicros_;  // micros() at last Fusion update (for dt)
    static bool     imuSamplePending_;  // true after a raw read, until Fusion consumes it
    static uint16_t imuTimingAvgUs_;
    static uint16_t imuTimingPeakUs_;
    static uint16_t imuTimingMaxUs_;

    // ---- Range sensors ----
    static UltrasonicDriver ultrasonics_[SENSOR_MAX_ULTRASONICS];
    static uint8_t          ultrasonicCount_;   // # that responded during init
    static bool             ultrasonicFound_[SENSOR_MAX_ULTRASONICS]; // per-slot init result
    static uint16_t         ultrasonicDistMm_[SENSOR_MAX_ULTRASONICS];
    static uint16_t         ultrasonicTimingAvgUs_;
    static uint16_t         ultrasonicTimingPeakUs_;
    static uint16_t         ultrasonicTimingMaxUs_;

    // ---- Voltages ----
    static float batteryVoltage_;
    static float rail5VVoltage_;
    static float servoVoltage_;

    // ---- Magnetometer calibration ----
    static MagCalData magCal_;
    static bool       magCalBackupValid_;
    static float      magCalBackupOffset_[3];
    static float      magCalBackupMatrix_[9];

    // ---- Statistics ----
    static bool     initialized_;
    static uint32_t updateCount_;
    static uint32_t lastUpdateTime_;

    // ---- ISR dispatch sub-tasks ----
    static void update100Hz();   // Alternating IMU/Fusion lane (100 Hz — every tick)
    static void update50Hz();    // Reserved medium-rate lane   ( 50 Hz — even ticks)
    static void update10Hz();    // Voltages only              ( 10 Hz — tick 0 only)

    // ---- Internal helpers ----
    static void initMagCalFromStorage();
    static void restoreMagCalibrationBackup();
    static void updateVoltages();
    static void updateMagCalSampling();
    static uint16_t readADCAverage(uint8_t pin, uint8_t numSamples = 4);
    static float    adcToVoltage(uint16_t adcValue, float dividerRatio);
};

#endif // SENSORMANAGER_H
