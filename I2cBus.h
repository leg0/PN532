#pragma once

#include <Wire.h>

namespace pn532
{

template <uint8_t Address, uint8_t Irq, uint8_t Reset>
struct I2cBus
{
	enum Status
	{
		Status_Busy,
		Status_Ready
	};

	static void begin()
	{
		Wire.begin();

		pinMode(Irq, INPUT);
		pinMode(Reset, OUTPUT);

		digitalWrite(Reset, HIGH);
		digitalWrite(Reset, LOW);
		delay(400);
		digitalWrite(Reset, HIGH);
	}

	/// Read single byte from bus.
	static uint8_t readByte()
	{
		return Wire.read();
	}

	/// Write single byte to bus.
	static void writeByte(uint8_t b)
	{
		Wire.write(b);
	}

	/// Write one byte and read one byte from bus.
	static uint8_t transceiveByte(uint8_t b); // TODO:

	/// Indicates that a write is coming.
	static uint8_t beginTransmission()
	{
		Wire.beginTransmission(Address);
	}

	static void endTransmission()
	{
		Wire.endTransmission();
	}

	/// Indicates that \a numberOfBytes bytes is going to be read.
	static uint8_t dataReadFollows(uint8_t numberOfBytes)
	{
		// Number of bytes that the caller wants, plus the leading byte we return from here.
		Wire.requestFrom(Address, numberOfBytes+1);

		// Return the leading byte of the response.
		return readByte();
	}

	static Status statusRead()
	{
		return (digitalRead(Irq) & 0x01) == 1
			? Status_Busy
			: Status_Ready;
	}

	/// Used by SPI.
	static void backupConfiguration() { /* NOP */ }

	/// Used by SPI.
	static void restoreConfiguration() { /* NOP */ }

	/// Used by SPI.
	static void select() { /* NOP */ }

	/// Used by SPI.
	static void deselect() { /* NOP */ }
};

}
