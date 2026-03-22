/**
 * @file PersistentStorage.h
 * @brief Non-volatile storage module using Arduino EEPROM
 *
 * The Arduino Mega 2560 has 4096 bytes (4 KB) of internal EEPROM.
 * EEPROM survives power-off and resets — data written once stays until
 * explicitly overwritten. The hardware supports approximately 100,000
 * write cycles per byte before wear-out, so avoid writing every loop().
 *
 * This module centralises all EEPROM access for the firmware. All other
 * modules that need persistent data should use this API instead of calling
 * EEPROM.put() / EEPROM.get() directly.
 *
 * EEPROM Layout (v3 — 56 bytes total, starting at address 0):
 * ─────────────────────────────────────────────────────────────
 *  Byte  0– 3  magic        uint32_t   0xDEAD2026 when layout is valid
 *  Byte  4     version      uint8_t    Layout version (currently 2)
 *  Byte  5     magCalValid  uint8_t    1 = valid mag calibration stored
 *  Byte  6– 7  reserved     uint8_t×2  Set to 0
 *  Byte  8–19  magOffset    float×3    Hard-iron offset XYZ (µT)
 *  Byte 20–55  magMatrix    float×9    Row-major 3x3 soft-iron correction matrix
 * ─────────────────────────────────────────────────────────────
 *
 * Wheel geometry is no longer stored in EEPROM. Edit WHEEL_DIAMETER_MM and
 * WHEEL_BASE_MM in RobotKinematics.h to match your robot's physical dimensions.
 *
 * Usage:
 *   PersistentStorage::init();   // call once in setup()
 *
 *   // Erase everything
 *   PersistentStorage::reset();
 */

#ifndef PERSISTENTSTORAGE_H
#define PERSISTENTSTORAGE_H

#include <Arduino.h>
#include <EEPROM.h>
#include <stdint.h>

// ============================================================================
// EEPROM LAYOUT CONSTANTS
// These are exposed so test sketches can read raw bytes for educational dumps.
// Production code should use the typed get/set methods below.
// ============================================================================

#define PS_MAGIC            0xDEAD2026UL
#define PS_VERSION          3
#define PS_BASE_ADDR        0

// Field offsets from PS_BASE_ADDR
#define PS_OFF_MAGIC        0   // uint32_t
#define PS_OFF_VERSION      4   // uint8_t
#define PS_OFF_MAG_VALID    5   // uint8_t
// 2 bytes reserved at 6–7
#define PS_OFF_MAG_X        8   // float
#define PS_OFF_MAG_Y        12  // float
#define PS_OFF_MAG_Z        16  // float
#define PS_OFF_MAG_MATRIX   20  // float[9]

#define PS_LAYOUT_SIZE      56  // total bytes used

// ============================================================================
// PERSISTENT STORAGE CLASS (Static)
// ============================================================================

class PersistentStorage {
public:
    /**
     * @brief Initialize and validate EEPROM storage
     *
     * Reads the magic number. If it matches, the stored layout is treated as
     * valid. If not (first boot or after reset()), the storage is initialised
     * with default values and the magic number is written.
     *
     * Always call this once in setup() before any get/set calls.
     */
    static void init();

    /**
     * @brief Erase all stored data and restore defaults
     *
     * Writes 0xFF over all managed bytes, then re-initialises with defaults.
     * Useful during development or to factory-reset a robot unit.
     */
    static void reset();

    /**
     * @brief Returns true if EEPROM contains a valid (non-corrupted) layout
     */
    static bool isValid() { return valid_; }

    /**
     * @brief Returns the stored layout version number
     */
    static uint8_t getVersion();

    // ========================================================================
    // MAGNETOMETER CALIBRATION
    // ========================================================================

    /**
     * @brief Load full mag calibration from EEPROM
     * @param ox, oy, oz Output — hard-iron offsets in µT
     * @param matrix Output — row-major 3x3 soft-iron correction matrix
     * @return False if no calibration has been saved
     */
    static bool getMagCalibration(float& ox, float& oy, float& oz, float matrix[9]);

    /**
     * @brief Persist full mag calibration to EEPROM
     * @param ox, oy, oz Hard-iron offsets in µT
     * @param matrix Row-major 3x3 soft-iron correction matrix
     */
    static void setMagCalibration(float ox, float oy, float oz, const float matrix[9]);

    /**
     * @brief Clear saved mag calibration (marks it invalid without erasing bytes)
     */
    static void clearMagCalibration();

    /**
     * @brief Returns true if a valid mag calibration is stored
     */
    static bool hasMagCalibration();

private:
    static bool valid_;     // true if magic number matched on init()

    // Write the magic + version header (marks storage as valid)
    static void writeHeader();
};

#endif // PERSISTENTSTORAGE_H
