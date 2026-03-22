#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include <Arduino.h>
#include <Print.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include "../config.h"

class DebugLogPort : public Print
{
public:
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    using Print::write;
};

extern DebugLogPort DEBUG_LOG;

class DebugLog
{
public:
    static void init();
    static void setPassthrough(bool enabled);
    static void flush();

    static void write(const char *text);
    static void writeLine(const char *text);
    static void writeFlash(const __FlashStringHelper *text);
    static void writeFlashLine(const __FlashStringHelper *text);
    static void printf_P(PGM_P format, ...);

    static uint16_t getQueuedBytes();
    static uint16_t getDroppedBytes() { return droppedBytes_; }

private:
    friend class DebugLogPort;
    static bool pushChar(char c);
    static void pushBuffer(const char *text);

    static char buffer_[DEBUG_LOG_BUFFER_SIZE];
    static char printfBuffer_[256];
    static uint16_t head_;
    static uint16_t tail_;
    static uint16_t droppedBytes_;
    static bool initialized_;
    static bool passthrough_;
};

#endif
