#include "StatusReporter.h"

#include <string.h>

#include "../config.h"
#include "../SystemManager.h"
#include "../utility.h"
#include "../drivers/DCMotor.h"
#include "DebugLog.h"
#include "LoopMonitor.h"
#include "MessageCenter.h"
#include "SafetyManager.h"
#include "SensorManager.h"
#include "UserIO.h"
#include "../drivers/ServoController.h"

extern DCMotor dcMotors[NUM_DC_MOTORS];

namespace {

constexpr uint8_t LIVEFLAG_CONTROL_MISS = 0x80U;

struct StatusSnapshot {
    SystemState state;
    uint8_t liveFlags;
    uint8_t latchedFlags;
    uint8_t loopFaultMask;
    uint8_t linkStateCode;
    uint8_t batteryStateCode;
    uint8_t servoRailStateCode;
    bool servoCtrlReady;
    bool servoOutputEnabled;
    bool servoI2CError;
    uint16_t vbatMv;
    uint16_t v5Mv;
    uint16_t vservoMv;
    uint32_t heartbeatAgeMs;
    uint16_t heartbeatTimeoutMs;
    uint32_t lastByteAgeMs;
    uint16_t servoEnabledMask;
    uint16_t freeRam;
    uint32_t txDroppedFrames;
    uint16_t debugDroppedBytes;
    uint16_t loopGapLastUs;
    uint16_t loopGapPeakUs;
    uint32_t motorRoundCount;
    uint32_t motorRequestedRound;
    uint32_t motorComputedRound;
    uint32_t motorAppliedRound;
    uint8_t motorSlot;
    uint8_t motorComputeSeq;
    uint8_t motorAppliedSeq;
    uint32_t missedRoundDelta;
    uint32_t lateComputeDelta;
    uint32_t reusedOutputDelta;
    uint32_t crossRoundDelta;
    bool motorComputeBusy;
    uint16_t pidAvgUs;
    uint16_t pidPeakUs;
    uint16_t pidMaxUs;
    uint16_t pidBudgetUs;
    uint16_t pidRoundAvgUs;
    uint16_t pidRoundPeakUs;
    uint16_t pidRoundMaxUs;
    uint16_t pidRoundBudgetUs;
    uint16_t stepAvgUs;
    uint16_t stepPeakUs;
    uint16_t stepMaxUs;
    uint16_t stepBudgetUs;
    uint16_t motorAvgUs;
    uint16_t motorPeakUs;
    uint16_t motorMaxUs;
    uint16_t motorBudgetUs;
    uint16_t sensorAvgUs;
    uint16_t sensorPeakUs;
    uint16_t sensorMaxUs;
    uint16_t sensorBudgetUs;
    uint16_t uartAvgUs;
    uint16_t uartPeakUs;
    uint16_t uartMaxUs;
    uint16_t uartBudgetUs;
    uint16_t ioAvgUs;
    uint16_t ioPeakUs;
    uint16_t ioMaxUs;
    uint16_t ioBudgetUs;
    uint16_t debugAvgUs;
    uint16_t debugPeakUs;
    uint16_t debugMaxUs;
    uint16_t flushAvgUs;
    uint16_t flushPeakUs;
    uint16_t flushMaxUs;
    uint16_t imuAvgUs;
    uint16_t imuPeakUs;
    uint16_t imuMaxUs;
    uint16_t ultrasonicAvgUs;
    uint16_t ultrasonicPeakUs;
    uint16_t ultrasonicMaxUs;
    bool imuAvailable;
    bool imuMagCalibrated;
    int16_t imuTempDeciC;
    int16_t imuRawAccZ;
    int16_t imuRawGyrZ;
    uint8_t ultrasonicConfiguredCount;
    uint8_t ultrasonicFoundCount;
    bool ultrasonicFound[SENSOR_MAX_ULTRASONICS];
    uint16_t ultrasonicDistMm[SENSOR_MAX_ULTRASONICS];
    uint16_t rxBytesWindow;
    uint16_t rxFramesWindow;
    uint16_t rxTlvsWindow;
    uint16_t rxHeartbeatsWindow;
    uint16_t txBytesWindow;
    uint16_t txFramesWindow;
    uint16_t uartRxErrors;
    uint32_t dor2Count;
    uint32_t fe2Count;
    uint16_t crcErrorCount;
    uint16_t frameLenErrorCount;
    uint16_t tlvErrorCount;
    uint16_t oversizeErrorCount;
    uint16_t rxAvailable;
    uint16_t rxPeak;
    uint16_t txPending;
    uint16_t txPeak;
};

StatusReporterUartFaultSnapshotFn g_uartFaultSnapshot = nullptr;
StatusReporterMotorControlSnapshotFn g_motorControlSnapshot = nullptr;

StatusSnapshot g_statusSnapshot = {};
bool g_statusSnapshotPending = false;
uint8_t g_statusSnapshotChunk = 0;

uint32_t g_lastLoopUs = 0;
uint16_t g_loopGapLastUs = 0;
uint16_t g_loopGapPeakUs = 0;
uint16_t g_rxBacklogPeak = 0;
uint16_t g_txPendingPeak = 0;
uint16_t g_statusTaskAvgUs = 0;
uint16_t g_statusTaskPeakUs = 0;
uint16_t g_statusTaskMaxUs = 0;
uint16_t g_flushAvgUs = 0;
uint16_t g_flushPeakUs = 0;
uint16_t g_flushMaxUs = 0;

void normalizeTimingTriplet(uint16_t avgUs,
                            uint16_t peakUs,
                            uint16_t maxUs,
                            uint16_t &displayPeakUs,
                            uint16_t &displayMaxUs) {
    displayPeakUs = (peakUs < avgUs) ? avgUs : peakUs;
    displayMaxUs = (maxUs < displayPeakUs) ? displayPeakUs : maxUs;
}

void recordAuxTiming(uint16_t elapsedUs,
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

const char *stateName(SystemState state) {
    switch (state) {
        case SystemState::SYS_STATE_INIT:    return "INIT";
        case SystemState::SYS_STATE_IDLE:    return "IDLE";
        case SystemState::SYS_STATE_RUNNING: return "RUNNING";
        case SystemState::SYS_STATE_ERROR:   return "ERROR";
        case SystemState::SYS_STATE_ESTOP:   return "ESTOP";
        default:                             return "?";
    }
}

void appendToken(char *buffer, size_t size, bool &first, const char *token) {
    size_t used = strlen(buffer);
    if (used >= (size - 1U) || token == nullptr || token[0] == '\0') {
        return;
    }
    if (!first) {
        strlcat(buffer, ",", size);
    }
    strlcat(buffer, token, size);
    first = false;
}

void formatErrorFlags(uint8_t flags, char *buffer, size_t size) {
    if (size == 0U) {
        return;
    }
    buffer[0] = '\0';
    bool first = true;
    if (flags & ERR_UNDERVOLTAGE)  appendToken(buffer, size, first, "UV");
    if (flags & ERR_OVERVOLTAGE)   appendToken(buffer, size, first, "OV");
    if (flags & ERR_ENCODER_FAIL)  appendToken(buffer, size, first, "ENC");
    if (flags & ERR_I2C_ERROR)     appendToken(buffer, size, first, "I2C");
    if (flags & ERR_IMU_ERROR)     appendToken(buffer, size, first, "IMU");
    if (flags & ERR_LIVENESS_LOST) appendToken(buffer, size, first, "HB");
    if (flags & ERR_LOOP_OVERRUN)     appendToken(buffer, size, first, "LOOP");
    if (flags & LIVEFLAG_CONTROL_MISS) appendToken(buffer, size, first, "CTRL");
    if (first) {
        strlcpy(buffer, "none", size);
    }
}

void formatLoopFaults(uint8_t mask, char *buffer, size_t size) {
    if (size == 0U) {
        return;
    }
    buffer[0] = '\0';
    bool first = true;
    if (mask & LOOP_FAULT_PID_ISR)     appendToken(buffer, size, first, "pid");
    if (mask & LOOP_FAULT_PID_ROUND)   appendToken(buffer, size, first, "pidr");
    if (mask & LOOP_FAULT_STEPPER_ISR) appendToken(buffer, size, first, "step");
    if (mask & LOOP_FAULT_MOTOR_TASK)  appendToken(buffer, size, first, "motor");
    if (mask & LOOP_FAULT_SENSOR_ISR)  appendToken(buffer, size, first, "sensor");
    if (mask & LOOP_FAULT_UART_TASK)   appendToken(buffer, size, first, "uart");
    if (mask & LOOP_FAULT_USERIO)      appendToken(buffer, size, first, "io");
    if (first) {
        strlcpy(buffer, "none", size);
    }
}

void formatRangeSummary(uint8_t configuredCount,
                        uint8_t foundCount,
                        const bool *found,
                        const uint16_t *distMm,
                        uint8_t maxCount,
                        char *buffer,
                        size_t size) {
    if (size == 0U) {
        return;
    }
    if (configuredCount == 0U) {
        strlcpy(buffer, "none", size);
        return;
    }

    snprintf_P(buffer, size, PSTR("cfg=%u found=%u"), configuredCount, foundCount);
    for (uint8_t i = 0; i < configuredCount && i < maxCount; i++) {
        char token[20];
        if (found[i]) {
            snprintf_P(token, sizeof(token), PSTR(" | %u=%umm"), i, distMm[i]);
        } else {
            snprintf_P(token, sizeof(token), PSTR(" | %u=missing"), i);
        }
        strlcat(buffer, token, size);
    }
}

uint8_t batteryStateCode() {
    if (!SensorManager::isBatteryPresent()) {
        return 4U;
    }
    if (SensorManager::isBatteryOvervoltage()) {
        return 3U;
    }
    if (SensorManager::isBatteryCritical()) {
        return 2U;
    }
    if (SensorManager::isBatteryLow()) {
        return 1U;
    }
    return 0U;
}

uint8_t servoRailStateCode() {
    float servoV = SensorManager::getServoVoltage();
    if (servoV < 1.0f) {
        return 1U;
    }
    if (servoV < VSERVO_MIN_PRESENT_V) {
        return 2U;
    }
    if (servoV > 9.0f) {
        return 3U;
    }
    return 0U;
}

const char *linkStateName(uint8_t linkStateCode) {
    switch (linkStateCode) {
        case 1: return "OK";
        case 2: return "LOST";
        default: return "IDLE";
    }
}

const char *batteryStateName(uint8_t batteryStateCode) {
    switch (batteryStateCode) {
        case 1: return "LOW";
        case 2: return "CRITICAL";
        case 3: return "HIGH";
        case 4: return "NO BATTERY";
        default: return "OK";
    }
}

const char *servoRailStateName(uint8_t servoRailStateCode) {
    switch (servoRailStateCode) {
        case 1: return "OFF";
        case 2: return "LOW";
        case 3: return "HIGH";
        default: return "OK";
    }
}

const char *systemHealthName(SystemState state, uint8_t liveFlags) {
    if (state == SystemState::SYS_STATE_ESTOP || state == SystemState::SYS_STATE_ERROR) {
        return "FAULT";
    }
    if (liveFlags != ERR_NONE) {
        return "WARN";
    }
    return "OK";
}

uint8_t computeLiveErrorFlags(SystemState state) {
    uint8_t flags = ERR_NONE;
    if (SensorManager::isBatteryLow())         flags |= ERR_UNDERVOLTAGE;
    if (SensorManager::isBatteryOvervoltage()) flags |= ERR_OVERVOLTAGE;
    if (!MessageCenter::isHeartbeatValid() && state == SystemState::SYS_STATE_RUNNING)
                                            flags |= ERR_LIVENESS_LOST;
    if (ServoController::hasI2CError())        flags |= ERR_I2C_ERROR;
    if (LoopMonitor::getLiveFaultMask() != 0U) flags |= ERR_LOOP_OVERRUN;
    for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
        if (dcMotors[i].isEncoderFailed()) {
            flags |= ERR_ENCODER_FAIL;
            break;
        }
    }
    return flags;
}

} // namespace

void StatusReporter::init(StatusReporterUartFaultSnapshotFn uartFaultSnapshot,
                          StatusReporterMotorControlSnapshotFn motorControlSnapshot) {
    g_uartFaultSnapshot = uartFaultSnapshot;
    g_motorControlSnapshot = motorControlSnapshot;
    g_statusSnapshotPending = false;
    g_statusSnapshotChunk = 0;
    g_lastLoopUs = 0;
    g_loopGapLastUs = 0;
    g_loopGapPeakUs = 0;
    g_rxBacklogPeak = 0;
    g_txPendingPeak = 0;
    g_statusTaskAvgUs = 0;
    g_statusTaskPeakUs = 0;
    g_statusTaskMaxUs = 0;
    g_flushAvgUs = 0;
    g_flushPeakUs = 0;
    g_flushMaxUs = 0;
}

void StatusReporter::recordLoopGap() {
#if !STATUS_REPORTER_ENABLED
    return;
#endif
    uint32_t nowUs = micros();
    if (g_lastLoopUs != 0U) {
        uint16_t gapUs = Utility::clampElapsedUs(nowUs - g_lastLoopUs);
        g_loopGapLastUs = gapUs;
        if (gapUs > g_loopGapPeakUs) {
            g_loopGapPeakUs = gapUs;
        }
    }
    g_lastLoopUs = nowUs;
}

void StatusReporter::updateWindowPeaks() {
#if !STATUS_REPORTER_ENABLED
    return;
#endif
    uint16_t rxBacklog = (uint16_t)RPI_SERIAL.available();
    if (rxBacklog > g_rxBacklogPeak) {
        g_rxBacklogPeak = rxBacklog;
    }

    uint16_t txPending = MessageCenter::getTxPendingBytes();
    if (txPending > g_txPendingPeak) {
        g_txPendingPeak = txPending;
    }
}

void StatusReporter::recordFlushTimingUs(uint16_t elapsedUs) {
#if !STATUS_REPORTER_ENABLED
    (void)elapsedUs;
    return;
#endif
    recordAuxTiming(elapsedUs, g_flushAvgUs, g_flushPeakUs, g_flushMaxUs);
}

void StatusReporter::task() {
#if !STATUS_REPORTER_ENABLED
    return;
#endif
    static uint32_t prevMissedRoundCount = 0;
    static uint32_t prevLateComputeCount = 0;
    static uint32_t prevReusedOutputCount = 0;
    static uint32_t prevCrossRoundComputeCount = 0;

    uint32_t t0 = micros();
    StatusSnapshot &snapshot = g_statusSnapshot;
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.state = SystemManager::getState();
    uint8_t faultMask = LoopMonitor::getLiveFaultMask();

    snapshot.vbatMv = (uint16_t)(SensorManager::getBatteryVoltage() * 1000.0f);
    snapshot.v5Mv = (uint16_t)(SensorManager::get5VRailVoltage() * 1000.0f);
    snapshot.vservoMv = (uint16_t)(SensorManager::getServoVoltage() * 1000.0f);
    snapshot.linkStateCode = (snapshot.state == SystemState::SYS_STATE_RUNNING)
                               ? (MessageCenter::isHeartbeatValid() ? 1U : 2U)
                               : 0U;
    snapshot.servoCtrlReady = ServoController::isInitialized();
    snapshot.servoOutputEnabled = ServoController::isEnabled();
    snapshot.servoI2CError = ServoController::hasI2CError();
    snapshot.batteryStateCode = batteryStateCode();
    snapshot.servoRailStateCode = servoRailStateCode();
    snapshot.latchedFlags = MessageCenter::getFaultLatchFlags();
    snapshot.loopFaultMask = faultMask;

    if (g_uartFaultSnapshot != nullptr) {
        g_uartFaultSnapshot(snapshot.dor2Count, snapshot.fe2Count);
    }
    MessageCenter::snapshotTrafficWindow(snapshot.rxBytesWindow,
                                         snapshot.rxFramesWindow,
                                         snapshot.rxTlvsWindow,
                                         snapshot.rxHeartbeatsWindow,
                                         snapshot.txBytesWindow,
                                         snapshot.txFramesWindow);
    if (g_motorControlSnapshot != nullptr) {
        g_motorControlSnapshot(snapshot.motorRoundCount,
                               snapshot.motorRequestedRound,
                               snapshot.motorComputedRound,
                               snapshot.motorAppliedRound,
                               snapshot.motorSlot,
                               snapshot.motorComputeSeq,
                               snapshot.motorAppliedSeq,
                               snapshot.missedRoundDelta,
                               snapshot.lateComputeDelta,
                               snapshot.reusedOutputDelta,
                               snapshot.crossRoundDelta,
                               snapshot.motorComputeBusy);
    }

    snapshot.missedRoundDelta -= prevMissedRoundCount;
    snapshot.lateComputeDelta -= prevLateComputeCount;
    snapshot.reusedOutputDelta -= prevReusedOutputCount;
    snapshot.crossRoundDelta -= prevCrossRoundComputeCount;
    prevMissedRoundCount += snapshot.missedRoundDelta;
    prevLateComputeCount += snapshot.lateComputeDelta;
    prevReusedOutputCount += snapshot.reusedOutputDelta;
    prevCrossRoundComputeCount += snapshot.crossRoundDelta;

    snapshot.liveFlags = computeLiveErrorFlags(snapshot.state);
    if (snapshot.missedRoundDelta != 0U ||
        snapshot.lateComputeDelta != 0U ||
        snapshot.reusedOutputDelta != 0U ||
        snapshot.crossRoundDelta != 0U) {
        snapshot.liveFlags |= LIVEFLAG_CONTROL_MISS;
    }

    snapshot.heartbeatAgeMs = MessageCenter::getTimeSinceHeartbeat();
    snapshot.heartbeatTimeoutMs = MessageCenter::getHeartbeatTimeoutMs();
    snapshot.lastByteAgeMs = MessageCenter::getTimeSinceRxByte();
    snapshot.servoEnabledMask = MessageCenter::getServoEnabledMask();
    snapshot.freeRam = MessageCenter::getFreeRAM();
    snapshot.txDroppedFrames = MessageCenter::getTxDroppedFrames();
    snapshot.debugDroppedBytes = DebugLog::getDroppedBytes();
    snapshot.loopGapLastUs = g_loopGapLastUs;
    snapshot.loopGapPeakUs = g_loopGapPeakUs;

    snapshot.pidAvgUs = LoopMonitor::getAvgUs(SLOT_PID_ISR);
    snapshot.pidPeakUs = LoopMonitor::getPeakUs(SLOT_PID_ISR);
    snapshot.pidMaxUs = LoopMonitor::getMaxUs(SLOT_PID_ISR);
    snapshot.pidBudgetUs = LoopMonitor::getBudgetUs(SLOT_PID_ISR);
    snapshot.pidRoundAvgUs = LoopMonitor::getPidRoundAvgUs();
    snapshot.pidRoundPeakUs = LoopMonitor::getPidRoundPeakUs();
    snapshot.pidRoundMaxUs = LoopMonitor::getPidRoundMaxUs();
    snapshot.pidRoundBudgetUs = LoopMonitor::getPidRoundBudgetUs();
    snapshot.stepAvgUs = LoopMonitor::getAvgUs(SLOT_STEPPER_ISR);
    snapshot.stepPeakUs = LoopMonitor::getPeakUs(SLOT_STEPPER_ISR);
    snapshot.stepMaxUs = LoopMonitor::getMaxUs(SLOT_STEPPER_ISR);
    snapshot.stepBudgetUs = LoopMonitor::getBudgetUs(SLOT_STEPPER_ISR);
    snapshot.motorAvgUs = LoopMonitor::getAvgUs(SLOT_MOTOR_TASK);
    snapshot.motorPeakUs = LoopMonitor::getPeakUs(SLOT_MOTOR_TASK);
    snapshot.motorMaxUs = LoopMonitor::getMaxUs(SLOT_MOTOR_TASK);
    snapshot.motorBudgetUs = LoopMonitor::getBudgetUs(SLOT_MOTOR_TASK);
    snapshot.sensorAvgUs = LoopMonitor::getAvgUs(SLOT_SENSOR_ISR);
    snapshot.sensorPeakUs = LoopMonitor::getPeakUs(SLOT_SENSOR_ISR);
    snapshot.sensorMaxUs = LoopMonitor::getMaxUs(SLOT_SENSOR_ISR);
    snapshot.sensorBudgetUs = LoopMonitor::getBudgetUs(SLOT_SENSOR_ISR);
    snapshot.ultrasonicAvgUs = SensorManager::getUltrasonicTimingAvgUs();
    snapshot.ultrasonicPeakUs = SensorManager::getUltrasonicTimingPeakUs();
    snapshot.ultrasonicMaxUs = SensorManager::getUltrasonicTimingMaxUs();
    snapshot.ultrasonicConfiguredCount = SensorManager::getUltrasonicConfiguredCount();
    snapshot.ultrasonicFoundCount = SensorManager::getUltrasonicCount();
    for (uint8_t i = 0; i < SENSOR_MAX_ULTRASONICS; i++) {
        snapshot.ultrasonicFound[i] = SensorManager::isUltrasonicFound(i);
        snapshot.ultrasonicDistMm[i] = SensorManager::getUltrasonicDistanceMm(i);
    }
    snapshot.uartAvgUs = LoopMonitor::getAvgUs(SLOT_UART_TASK);
    snapshot.uartPeakUs = LoopMonitor::getPeakUs(SLOT_UART_TASK);
    snapshot.uartMaxUs = LoopMonitor::getMaxUs(SLOT_UART_TASK);
    snapshot.uartBudgetUs = LoopMonitor::getBudgetUs(SLOT_UART_TASK);
    snapshot.ioAvgUs = LoopMonitor::getAvgUs(SLOT_USERIO);
    snapshot.ioPeakUs = LoopMonitor::getPeakUs(SLOT_USERIO);
    snapshot.ioMaxUs = LoopMonitor::getMaxUs(SLOT_USERIO);
    snapshot.ioBudgetUs = LoopMonitor::getBudgetUs(SLOT_USERIO);
    snapshot.debugAvgUs = g_statusTaskAvgUs;
    snapshot.debugPeakUs = g_statusTaskPeakUs;
    snapshot.debugMaxUs = g_statusTaskMaxUs;
    snapshot.flushAvgUs = g_flushAvgUs;
    snapshot.flushPeakUs = g_flushPeakUs;
    snapshot.flushMaxUs = g_flushMaxUs;
    snapshot.imuAvgUs = SensorManager::getImuTimingAvgUs();
    snapshot.imuPeakUs = SensorManager::getImuTimingPeakUs();
    snapshot.imuMaxUs = SensorManager::getImuTimingMaxUs();
    snapshot.imuAvailable = SensorManager::isIMUAvailable();
    snapshot.imuMagCalibrated = SensorManager::isMagCalibrated();
    snapshot.imuTempDeciC = SensorManager::getRawTempDeciC();
    snapshot.imuRawAccZ = SensorManager::getRawAccZ();
    snapshot.imuRawGyrZ = SensorManager::getRawGyrZ();
    snapshot.uartRxErrors = MessageCenter::getUartRxErrors();
    snapshot.crcErrorCount = MessageCenter::getCrcErrorCount();
    snapshot.frameLenErrorCount = MessageCenter::getFrameLenErrorCount();
    snapshot.tlvErrorCount = MessageCenter::getTlvErrorCount();
    snapshot.oversizeErrorCount = MessageCenter::getOversizeErrorCount();
    snapshot.rxAvailable = (uint16_t)RPI_SERIAL.available();
    snapshot.rxPeak = g_rxBacklogPeak;
    snapshot.txPending = MessageCenter::getTxPendingBytes();
    snapshot.txPeak = g_txPendingPeak;

    g_statusSnapshotPending = true;
    g_statusSnapshotChunk = 0;

    LoopMonitor::clearPeaks();
    SensorManager::clearTimingPeaks();
    g_loopGapPeakUs = g_loopGapLastUs;
    g_rxBacklogPeak = snapshot.rxAvailable;
    g_txPendingPeak = snapshot.txPending;
    g_statusTaskPeakUs = 0;
    g_flushPeakUs = 0;

    recordAuxTiming(Utility::clampElapsedUs(micros() - t0), g_statusTaskAvgUs, g_statusTaskPeakUs, g_statusTaskMaxUs);
}

void StatusReporter::emitChunk() {
#if !STATUS_REPORTER_ENABLED
    return;
#endif
    if (!g_statusSnapshotPending) {
        return;
    }
    if (DebugLog::getQueuedBytes() > (DEBUG_LOG_BUFFER_SIZE / 2U)) {
        return;
    }

    uint32_t t0 = micros();
    const StatusSnapshot &s = g_statusSnapshot;

    switch (g_statusSnapshotChunk) {
        case 0:
            DebugLog::printf_P(PSTR("\n[SYSTEM]\n"));
            break;
        case 1:
            DebugLog::printf_P(PSTR("State: %s [%s]\n"),
                               stateName(s.state),
                               systemHealthName(s.state, s.liveFlags));
            break;
        case 2:
            DebugLog::printf_P(PSTR("Link: %s | Heartbeat: %lums/%ums | Last Byte: %lums\n"),
                               linkStateName(s.linkStateCode),
                               (unsigned long)s.heartbeatAgeMs,
                               s.heartbeatTimeoutMs,
                               (unsigned long)s.lastByteAgeMs);
            break;
        case 3:
            DebugLog::printf_P(PSTR("Battery: %u.%03uV [%s] | Servo: %u.%03uV [%s] | 5V: %u.%03uV\n"),
                               (unsigned)(s.vbatMv / 1000U), (unsigned)(s.vbatMv % 1000U), batteryStateName(s.batteryStateCode),
                               (unsigned)(s.vservoMv / 1000U), (unsigned)(s.vservoMv % 1000U), servoRailStateName(s.servoRailStateCode),
                               (unsigned)(s.v5Mv / 1000U), (unsigned)(s.v5Mv % 1000U));
            break;
        case 4:
            DebugLog::printf_P(PSTR("ServoCtrl: %s [%s] | EnabledMask: 0x%04X | I2C: %s\n"),
                               s.servoCtrlReady ? "READY" : "OFFLINE",
                               s.servoOutputEnabled ? "ENABLED" : "DISABLED",
                               s.servoEnabledMask,
                               s.servoI2CError ? "ERROR" : "OK");
            break;
        case 5:
        {
            char liveBuf[28];
            char latchedBuf[28];
            char loopBuf[36];
            formatErrorFlags(s.liveFlags, liveBuf, sizeof(liveBuf));
            formatErrorFlags(s.latchedFlags, latchedBuf, sizeof(latchedBuf));
            formatLoopFaults(s.loopFaultMask, loopBuf, sizeof(loopBuf));
            DebugLog::printf_P(PSTR("Faults: live=%s | latched=%s | overrun=%s\n"),
                               liveBuf, latchedBuf, loopBuf);
            break;
        }
        case 6:
            DebugLog::printf_P(PSTR("Control: round=%lu | req/cmp/app=%lu/%lu/%lu | slot=%u | prep/app=%u/%u | pipe=%u | busy=%s\n"),
                               (unsigned long)s.motorRoundCount,
                               (unsigned long)s.motorRequestedRound,
                               (unsigned long)s.motorComputedRound,
                               (unsigned long)s.motorAppliedRound,
                               s.motorSlot,
                               s.motorComputeSeq,
                               s.motorAppliedSeq,
                               (uint8_t)(s.motorComputeSeq - s.motorAppliedSeq),
                               s.motorComputeBusy ? "yes" : "no");
            break;
        case 7:
            DebugLog::printf_P(PSTR("ControlWin: missed=%lu | late=%lu | reused=%lu | cross=%lu\n"),
                               (unsigned long)s.missedRoundDelta,
                               (unsigned long)s.lateComputeDelta,
                               (unsigned long)s.reusedOutputDelta,
                               (unsigned long)s.crossRoundDelta);
            break;
        case 8:
            DebugLog::printf_P(PSTR("Memory: RAM=%u B | TxDrop=%lu | DbgDrop=%u | LoopGap=%u/%uus\n"),
                               s.freeRam,
                               (unsigned long)s.txDroppedFrames,
                               s.debugDroppedBytes,
                               s.loopGapLastUs,
                               s.loopGapPeakUs);
            break;
        case 9:
            DebugLog::printf_P(PSTR("\n[TIMING] (avg/peak/max)\n"));
            break;
        case 10:
        {
            uint16_t pidPeakUs;
            uint16_t pidMaxUs;
            uint16_t pidRoundPeakUs;
            uint16_t pidRoundMaxUs;
            uint16_t stepPeakUs;
            uint16_t stepMaxUs;
            normalizeTimingTriplet(s.pidAvgUs, s.pidPeakUs, s.pidMaxUs, pidPeakUs, pidMaxUs);
            normalizeTimingTriplet(s.pidRoundAvgUs, s.pidRoundPeakUs, s.pidRoundMaxUs, pidRoundPeakUs, pidRoundMaxUs);
            normalizeTimingTriplet(s.stepAvgUs, s.stepPeakUs, s.stepMaxUs, stepPeakUs, stepMaxUs);
            DebugLog::printf_P(PSTR("pid %u/%u/%u (%u) us | pidr %u/%u/%u (%u) us | step %u/%u/%u (%u) us\n"),
                               s.pidAvgUs, pidPeakUs, pidMaxUs, s.pidBudgetUs,
                               s.pidRoundAvgUs, pidRoundPeakUs, pidRoundMaxUs, s.pidRoundBudgetUs,
                               s.stepAvgUs, stepPeakUs, stepMaxUs, s.stepBudgetUs);
            break;
        }
        case 11:
        {
            uint16_t motorPeakUs;
            uint16_t motorMaxUs;
            uint16_t sensorPeakUs;
            uint16_t sensorMaxUs;
            normalizeTimingTriplet(s.motorAvgUs, s.motorPeakUs, s.motorMaxUs, motorPeakUs, motorMaxUs);
            normalizeTimingTriplet(s.sensorAvgUs, s.sensorPeakUs, s.sensorMaxUs, sensorPeakUs, sensorMaxUs);
            DebugLog::printf_P(PSTR("motor %u/%u/%u (%u) us | sensor %u/%u/%u (%u) us\n"),
                               s.motorAvgUs, motorPeakUs, motorMaxUs, s.motorBudgetUs,
                               s.sensorAvgUs, sensorPeakUs, sensorMaxUs, s.sensorBudgetUs);
            break;
        }
        case 12:
        {
            uint16_t uartPeakUs;
            uint16_t uartMaxUs;
            uint16_t ioPeakUs;
            uint16_t ioMaxUs;
            normalizeTimingTriplet(s.uartAvgUs, s.uartPeakUs, s.uartMaxUs, uartPeakUs, uartMaxUs);
            normalizeTimingTriplet(s.ioAvgUs, s.ioPeakUs, s.ioMaxUs, ioPeakUs, ioMaxUs);
            DebugLog::printf_P(PSTR("uart %u/%u/%u (%u) us | io %u/%u/%u (%u) us\n"),
                               s.uartAvgUs, uartPeakUs, uartMaxUs, s.uartBudgetUs,
                               s.ioAvgUs, ioPeakUs, ioMaxUs, s.ioBudgetUs);
            break;
        }
        case 13:
        {
            uint16_t debugPeakUs;
            uint16_t debugMaxUs;
            uint16_t flushPeakUs;
            uint16_t flushMaxUs;
            normalizeTimingTriplet(s.debugAvgUs, s.debugPeakUs, s.debugMaxUs, debugPeakUs, debugMaxUs);
            normalizeTimingTriplet(s.flushAvgUs, s.flushPeakUs, s.flushMaxUs, flushPeakUs, flushMaxUs);
            DebugLog::printf_P(PSTR("debug %u/%u/%u us | flush %u/%u/%u us\n"),
                               s.debugAvgUs, debugPeakUs, debugMaxUs,
                               s.flushAvgUs, flushPeakUs, flushMaxUs);
            break;
        }
        case 14:
        {
            uint16_t imuPeakUs;
            uint16_t imuMaxUs;
            uint16_t ultrasonicPeakUs;
            uint16_t ultrasonicMaxUs;
            normalizeTimingTriplet(s.imuAvgUs, s.imuPeakUs, s.imuMaxUs, imuPeakUs, imuMaxUs);
            normalizeTimingTriplet(s.ultrasonicAvgUs, s.ultrasonicPeakUs, s.ultrasonicMaxUs, ultrasonicPeakUs, ultrasonicMaxUs);
            DebugLog::printf_P(PSTR("imu %u/%u/%u (%u) us | ultra %u/%u/%u (%u) us\n"),
                               s.imuAvgUs, imuPeakUs, imuMaxUs, s.sensorBudgetUs,
                               s.ultrasonicAvgUs, ultrasonicPeakUs, ultrasonicMaxUs, s.sensorBudgetUs);
            break;
        }
        case 15:
            DebugLog::printf_P(PSTR("\n[SENSORS]\n"));
            break;
        case 16:
        {
            char imuSummary[88];
            snprintf_P(imuSummary,
                       sizeof(imuSummary),
                       PSTR("avail=%s | mag=%s | temp=%d.%uC | az=%dmg | gz=%d.%udps"),
                       s.imuAvailable ? "1" : "0",
                       s.imuMagCalibrated ? "9DOF" : "6DOF",
                       (int)(s.imuTempDeciC / 10),
                       (unsigned)abs(s.imuTempDeciC % 10),
                       (int)s.imuRawAccZ,
                       (int)(s.imuRawGyrZ / 10),
                       (unsigned)abs(s.imuRawGyrZ % 10));
            DebugLog::printf_P(PSTR("IMU: %s\n"), imuSummary);
            break;
        }
        case 17:
        {
            char ultrasonicSummary[112];
            formatRangeSummary(s.ultrasonicConfiguredCount,
                               s.ultrasonicFoundCount,
                               s.ultrasonicFound,
                               s.ultrasonicDistMm,
                               SENSOR_MAX_ULTRASONICS,
                               ultrasonicSummary,
                               sizeof(ultrasonicSummary));
            DebugLog::printf_P(PSTR("Ultrasonic: %s\n"), ultrasonicSummary);
            break;
        }
        case 18:
            DebugLog::printf_P(PSTR("\n[UART]\n"));
            break;
        case 19:
            DebugLog::printf_P(PSTR("activity rxB/frame/tlv/hb=%u/%u/%u/%u txB/frame=%u/%u\n"),
                               s.rxBytesWindow, s.rxFramesWindow, s.rxTlvsWindow, s.rxHeartbeatsWindow,
                               s.txBytesWindow, s.txFramesWindow);
            break;
        default:
            DebugLog::printf_P(PSTR("total_err=%u dor=%lu fe=%lu crc=%u frame=%u tlv=%u oversize=%u rxQ(available/peak)=%u/%u txQ(pending/peak)=%u/%u\n"),
                               s.uartRxErrors,
                               (unsigned long)s.dor2Count,
                               (unsigned long)s.fe2Count,
                               s.crcErrorCount,
                               s.frameLenErrorCount,
                               s.tlvErrorCount,
                               s.oversizeErrorCount,
                               s.rxAvailable,
                               s.rxPeak,
                               s.txPending,
                               s.txPeak);
            g_statusSnapshotPending = false;
            g_statusSnapshotChunk = 0;
            recordAuxTiming(Utility::clampElapsedUs(micros() - t0), g_statusTaskAvgUs, g_statusTaskPeakUs, g_statusTaskMaxUs);
            return;
    }

    g_statusSnapshotChunk++;
    recordAuxTiming(Utility::clampElapsedUs(micros() - t0), g_statusTaskAvgUs, g_statusTaskPeakUs, g_statusTaskMaxUs);
}
