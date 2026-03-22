#include "utility.h"
#include "config.h"

namespace Utility {

namespace {
bool g_prevUart2Dor = false;
bool g_prevUart2Fe = false;
} // namespace

uint16_t clampElapsedUs(uint32_t elapsedUs) {
    return (elapsedUs > 0xFFFFUL) ? 0xFFFFU : (uint16_t)elapsedUs;
}

uint16_t readTimer1CounterTicks() {
    return TCNT1;
}

uint16_t readTimer3CounterTicks() {
    return TCNT3;
}

uint16_t timerTicksToUs(uint16_t ticks) {
    return (uint16_t)((ticks + 1U) >> 1);
}

Uart2FaultEdges sampleUart2FaultEdges() {
    uint8_t status = UCSR2A;
    bool dorActive = (status & _BV(DOR2)) != 0;
    bool feActive = (status & _BV(FE2)) != 0;

    Uart2FaultEdges edges = {
        dorActive && !g_prevUart2Dor,
        feActive && !g_prevUart2Fe,
    };

    g_prevUart2Dor = dorActive;
    g_prevUart2Fe = feActive;
    return edges;
}

void printStartupBanner() {
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("========================================"));
    DEBUG_SERIAL.println(F("  MAE 162 Robot Firmware v0.9.5"));
    DEBUG_SERIAL.println(F("========================================"));
    DEBUG_SERIAL.println();
}

void initDebugPins() {
#if DEBUG_PINS_ENABLED
    pinMode(DEBUG_PIN_ENCODER_ISR, OUTPUT);  // A7
    pinMode(DEBUG_PIN_STEPPER_ISR, OUTPUT);  // A8
    pinMode(DEBUG_PIN_UART_LATE,   OUTPUT);  // A9
    pinMode(DEBUG_PIN_PID_LOOP,    OUTPUT);  // A10
    pinMode(DEBUG_PIN_UART_TASK,   OUTPUT);  // A11
    pinMode(DEBUG_PIN_UART_RX,     OUTPUT);  // A12
    pinMode(DEBUG_PIN_UART_TX,     OUTPUT);  // A13
    digitalWrite(DEBUG_PIN_ENCODER_ISR, LOW);
    digitalWrite(DEBUG_PIN_STEPPER_ISR, LOW);
    digitalWrite(DEBUG_PIN_UART_LATE,   LOW);
    digitalWrite(DEBUG_PIN_PID_LOOP,    LOW);
    digitalWrite(DEBUG_PIN_UART_TASK,   LOW);
    digitalWrite(DEBUG_PIN_UART_RX,     LOW);
    digitalWrite(DEBUG_PIN_UART_TX,     LOW);
    DEBUG_SERIAL.println(F("[Setup] Debug pins configured (A7-A13)"));
#endif
}

void printStartupSummary() {
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("[Setup] Initialization complete!"));
    DEBUG_SERIAL.println(F("  Hard RT: DC slot@800Hz + Stepper@10kHz"));
    DEBUG_SERIAL.print(F("  Soft RT: UART@"));
    DEBUG_SERIAL.print(UART_COMMS_FREQ_HZ);
    DEBUG_SERIAL.print(F("Hz, Safety@"));
    DEBUG_SERIAL.print(SENSOR_UPDATE_FREQ_HZ);
    DEBUG_SERIAL.print(F("Hz, MotorCompute@"));
    DEBUG_SERIAL.print(MOTOR_UPDATE_FREQ_HZ);
    DEBUG_SERIAL.print(F("Hz(round-flag), Sensors@"));
    DEBUG_SERIAL.print(SENSOR_UPDATE_FREQ_HZ);
    DEBUG_SERIAL.print(F("Hz, UserIO@"));
    DEBUG_SERIAL.print(USER_IO_FREQ_HZ);
    DEBUG_SERIAL.print(F("Hz, Status@"));
#if STATUS_REPORTER_ENABLED
    DEBUG_SERIAL.print(STATUS_REPORT_HZ);
    DEBUG_SERIAL.println(F("Hz"));
#else
    DEBUG_SERIAL.println(F("off"));
#endif
    DEBUG_SERIAL.println(F("[Setup] Entered main loop."));
    DEBUG_SERIAL.println();
}

} // namespace Utility
