/**
 * @file SafetyManager.cpp
 * @brief Hard real-time safety watchdog implementation
 */

#include "SafetyManager.h"
#include "MessageCenter.h"
#include "SensorManager.h"
#include "../SystemManager.h"

void SafetyManager::check() {
    SystemState state = SystemManager::getState();

    // Already in a safe or terminal state — nothing to do (common path on boot)
    if (state == SYS_STATE_ERROR ||
        state == SYS_STATE_ESTOP ||
        state == SYS_STATE_INIT) {
        return;
    }

    // ── Fault detection ───────────────────────────────────────────────────────

    // Heartbeat loss is only a fault while RUNNING:
    // In IDLE, no RPi contact is expected (robot may be tethered or stopped).
    bool heartbeatFault = SystemManager::shouldTripHeartbeatFault() &&
                          !MessageCenter::isHeartbeatValid();

    // Battery faults are armed and interpreted by SystemManager.
    bool batteryFault = SystemManager::shouldTripBatteryFault();

    // Common case: all OK — return immediately
    if (!heartbeatFault && !batteryFault) return;

    // ── Fault response ────────────────────────────────────────────────────────

    // Build the trigger bitmask before changing state so the original cause
    // is preserved in the latched SYS_STATE / SYS_DIAG fault view.
    uint8_t triggerFlags = 0;
    if (heartbeatFault)                      triggerFlags |= ERR_LIVENESS_LOST;
    triggerFlags |= SystemManager::getBatteryFaultFlags();

    // Let SystemManager own the ERROR transition policy and shutdown sequence.
    SystemManager::triggerSafetyFaultFromIsr(triggerFlags);
}
