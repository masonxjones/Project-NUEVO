/**
 * @file PersistentStorage.cpp
 * @brief Implementation of EEPROM-backed persistent storage
 *
 * See PersistentStorage.h for the full EEPROM layout and usage notes.
 *
 * Key Arduino EEPROM facts for students:
 *  - EEPROM.put() only writes bytes that have changed (automatic wear-leveling)
 *  - EEPROM.get() reads without any wear effect — read as often as needed
 *  - Arduino Mega 2560: 4096 bytes available at addresses 0x000–0xFFF
 *  - ~100,000 write cycles per byte before wear-out
 *    → at 1 write/second that's ~27 hours; at 1 write/minute that's ~69 days
 *    → never write to EEPROM inside loop() without rate-limiting
 */

#include "PersistentStorage.h"
#include <math.h>

namespace {

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
// STATIC MEMBER INITIALIZATION
// ============================================================================

bool PersistentStorage::valid_ = false;

// ============================================================================
// INIT / RESET
// ============================================================================

void PersistentStorage::init() {
    uint32_t magic = 0;
    uint8_t version = 0;
    EEPROM.get(PS_BASE_ADDR + PS_OFF_MAGIC, magic);
    EEPROM.get(PS_BASE_ADDR + PS_OFF_VERSION, version);

    if (magic == PS_MAGIC && version == PS_VERSION) {
        valid_ = true;
        return;
    }

    // First boot or layout upgrade: write header + defaults.
    writeHeader();

    // No mag calibration by default.
    float zero = 0.0f;
    float identity[9];
    setIdentityMatrix(identity);
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAG_VALID, (uint8_t)0);
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAG_X, zero);
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAG_Y, zero);
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAG_Z, zero);
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAG_MATRIX, identity);

    valid_ = true;
}

void PersistentStorage::reset() {
    // Invalidate by writing 0xFF over magic; EEPROM.update only writes changed bytes
    uint32_t invalid = 0xFFFFFFFFUL;
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAGIC, invalid);

    // Zero all managed bytes
    for (int i = 1; i < PS_LAYOUT_SIZE; i++) {
        EEPROM.update(PS_BASE_ADDR + i, 0x00);
    }

    // Re-initialise with defaults
    valid_ = false;
    init();
}

// ============================================================================
// VERSION
// ============================================================================

uint8_t PersistentStorage::getVersion() {
    if (!valid_) return 0;
    uint8_t v = 0;
    EEPROM.get(PS_BASE_ADDR + PS_OFF_VERSION, v);
    return v;
}

// ============================================================================
// MAGNETOMETER CALIBRATION
// ============================================================================

bool PersistentStorage::getMagCalibration(float& ox, float& oy, float& oz, float matrix[9]) {
    if (!valid_) return false;
    uint8_t valid = 0;
    EEPROM.get(PS_BASE_ADDR + PS_OFF_MAG_VALID, valid);
    if (valid != 1) return false;
    EEPROM.get(PS_BASE_ADDR + PS_OFF_MAG_X, ox);
    EEPROM.get(PS_BASE_ADDR + PS_OFF_MAG_Y, oy);
    EEPROM.get(PS_BASE_ADDR + PS_OFF_MAG_Z, oz);
    for (uint8_t i = 0; i < 9; i++) {
        EEPROM.get(PS_BASE_ADDR + PS_OFF_MAG_MATRIX + (int)(i * sizeof(float)), matrix[i]);
        if (!isfinite(matrix[i])) {
            return false;
        }
    }
    return true;
}

void PersistentStorage::setMagCalibration(float ox, float oy, float oz, const float matrix[9]) {
    if (!valid_) return;
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAG_X, ox);
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAG_Y, oy);
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAG_Z, oz);
    for (uint8_t i = 0; i < 9; i++) {
        EEPROM.put(PS_BASE_ADDR + PS_OFF_MAG_MATRIX + (int)(i * sizeof(float)), matrix[i]);
    }
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAG_VALID, (uint8_t)1);  // mark valid last
}

void PersistentStorage::clearMagCalibration() {
    if (!valid_) return;
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAG_VALID, (uint8_t)0);
}

bool PersistentStorage::hasMagCalibration() {
    if (!valid_) return false;
    uint8_t v = 0;
    EEPROM.get(PS_BASE_ADDR + PS_OFF_MAG_VALID, v);
    return (v == 1);
}

// ============================================================================
// INTERNAL
// ============================================================================

void PersistentStorage::writeHeader() {
    uint32_t magic   = PS_MAGIC;
    uint8_t  version = PS_VERSION;
    EEPROM.put(PS_BASE_ADDR + PS_OFF_MAGIC,   magic);
    EEPROM.put(PS_BASE_ADDR + PS_OFF_VERSION, version);
}
