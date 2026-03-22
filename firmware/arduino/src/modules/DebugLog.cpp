#include "DebugLog.h"
#include "../config.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

DebugLogPort DEBUG_LOG;
HardwareSerial &DEBUG_SERIAL_PORT = Serial;
Print &DEBUG_SERIAL = DEBUG_LOG;

char DebugLog::buffer_[DEBUG_LOG_BUFFER_SIZE];
char DebugLog::printfBuffer_[256];
uint16_t DebugLog::head_ = 0;
uint16_t DebugLog::tail_ = 0;
uint16_t DebugLog::droppedBytes_ = 0;
bool DebugLog::initialized_ = false;
bool DebugLog::passthrough_ = false;

size_t DebugLogPort::write(uint8_t c)
{
    if (DebugLog::passthrough_) {
        return DEBUG_SERIAL_PORT.write(c);
    }
    return DebugLog::pushChar((char)c) ? 1U : 0U;
}

size_t DebugLogPort::write(const uint8_t *buffer, size_t size)
{
    if (buffer == nullptr) {
        return 0;
    }
    if (DebugLog::passthrough_) {
        return DEBUG_SERIAL_PORT.write(buffer, size);
    }
    if (!DebugLog::initialized_) {
        return 0;
    }

    size_t written = 0;
    while (written < size) {
        if (!DebugLog::pushChar((char)buffer[written])) {
            break;
        }
        written++;
    }
    return written;
}

void DebugLog::init()
{
    head_ = 0;
    tail_ = 0;
    droppedBytes_ = 0;
    initialized_ = true;
    passthrough_ = false;
}

void DebugLog::setPassthrough(bool enabled)
{
    passthrough_ = enabled;
}

bool DebugLog::pushChar(char c)
{
    uint16_t next = (uint16_t)((head_ + 1U) % DEBUG_LOG_BUFFER_SIZE);
    if (next == tail_) {
        if (droppedBytes_ < 0xFFFF) {
            droppedBytes_++;
        }
        return false;
    }

    buffer_[head_] = c;
    head_ = next;
    return true;
}

void DebugLog::pushBuffer(const char *text)
{
    if (!initialized_ || text == nullptr) {
        return;
    }

    while (*text != '\0') {
        if (!pushChar(*text++)) {
            return;
        }
    }
}

void DebugLog::write(const char *text)
{
    pushBuffer(text);
}

void DebugLog::writeLine(const char *text)
{
    pushBuffer(text);
    pushChar('\n');
}

void DebugLog::writeFlash(const __FlashStringHelper *text)
{
    if (!initialized_ || text == nullptr) {
        return;
    }

    PGM_P cursor = reinterpret_cast<PGM_P>(text);
    char c = pgm_read_byte(cursor++);
    while (c != '\0') {
        if (!pushChar(c)) {
            return;
        }
        c = pgm_read_byte(cursor++);
    }
}

void DebugLog::writeFlashLine(const __FlashStringHelper *text)
{
    writeFlash(text);
    pushChar('\n');
}

void DebugLog::printf_P(PGM_P format, ...)
{
    if (!initialized_ || format == nullptr) {
        return;
    }

    va_list args;
    va_start(args, format);
    vsnprintf_P(printfBuffer_, sizeof(printfBuffer_), format, args);
    va_end(args);
    pushBuffer(printfBuffer_);
}

void DebugLog::flush()
{
    if (!initialized_) {
        return;
    }

    int space = DEBUG_SERIAL_PORT.availableForWrite();
    uint8_t budget = DEBUG_FLUSH_MAX_BYTES_PER_PASS;
    while (space > 0 && tail_ != head_ && budget > 0U) {
        DEBUG_SERIAL_PORT.write((uint8_t)buffer_[tail_]);
        tail_ = (uint16_t)((tail_ + 1U) % DEBUG_LOG_BUFFER_SIZE);
        space--;
        budget--;
    }
}

uint16_t DebugLog::getQueuedBytes()
{
    if (head_ >= tail_) {
        return (uint16_t)(head_ - tail_);
    }
    return (uint16_t)(DEBUG_LOG_BUFFER_SIZE - tail_ + head_);
}
