#pragma once

#include <Wire.h>

namespace pn532
{

template <uint8_t Address>
struct I2cBus
{
	static void begin() { Wire.begin(Address); }

	/// Read single byte from bus.
	static uint8_t readByte(); // TODO:

	/// Write single byte to bus.
	static void writeByte(uint8_t b); // TODO:

	/// Write one byte and read one byte from bus.
	static uint8_t transceiveByte(uint8_t b); // TODO:

    // In SPI mode, a byte must be sent to indicate what the next bytes are going to be.
    // In other modes, these may be implemented as NOP.
    static uint8_t dataWriteFollows(); // TODO:
    static uint8_t dataReadFollows(); // TODO:
    //static uint8_t statusReadFollows();

	static void backupConfiguration() { }
	static void restoreConfiguration() { }

	static void select() { }
	static void deselect() { }
};

}
