#pragma once

#include <Arduino.h>

namespace pn532
{

struct SpiBusBase
{
	static void begin();

	static uint8_t readByte();
	static void writeByte(uint8_t b);
	static uint8_t transceiveByte(uint8_t b);

    // In SPI mode, a byte must be sent to indicate what the next bytes are going to be.
    // In other modes, these may be implemented as NOP.
    static uint8_t dataWriteFollows();
    static uint8_t dataReadFollows();
    //static uint8_t statusReadFollows();


	static void backupConfiguration();
	static void restoreConfiguration();

	static void select();
	static void deselect();
};

template <uint8_t CS, bool HighSelects = false>
struct SpiBus : SpiBusBase
{
	static uint8_t const ChipSelect = CS;
	static void begin()
	{
		SpiBusBase::begin();
		pinMode(ChipSelect, OUTPUT);
	}

	static void select()
	{
		digitalWrite(ChipSelect,  HighSelects ? HIGH : LOW);
	}

	static void deselect()
	{
		digitalWrite(ChipSelect, !HighSelects ? LOW : HIGH);
	}
};

} // pn532
