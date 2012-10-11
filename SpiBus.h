#pragma once

#include <Arduino.h>

namespace pn532
{

// This namespace contains implementation details. You don't want to use stuff
// from here directly.
namespace detail {
struct SpiBusBase
{
    enum { Busy, Ready };

    static void begin();

    static uint8_t readByte();
    static void writeByte(uint8_t b);
    static uint8_t transceiveByte(uint8_t b);

    static uint8_t beginTransmission();
    /// Used by I2C.
    static void endTransmission() { /* NOP */ }
    static uint8_t dataReadFollows(uint8_t bytesToRead);
    static uint8_t readStatus();

    static void backupConfiguration();
    static void restoreConfiguration();
};
} // detail

template <uint8_t CS, bool HighSelects = false>
struct SpiBus : detail::SpiBusBase
{
    static uint8_t const ChipSelect = CS;
    static void begin()
    {
        SpiBusBase::begin();
        pinMode(ChipSelect, OUTPUT);
    }

    static void select()
    {
        digitalWrite(ChipSelect, HighSelects ? HIGH : LOW);
    }

    static void deselect()
    {
        digitalWrite(ChipSelect, HighSelects ? LOW : HIGH);
    }
};

} // pn532
