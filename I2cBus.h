#pragma once

#include <Wire.h>

namespace pn532
{

template <uint8_t Address, uint8_t Irq, uint8_t Reset>
struct I2cBus
{
	enum { Busy, Ready };

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
		return Wire.requestFrom(Address, numberOfBytes);
	}

	static uint8_t statusRead()
	{
		return digitalRead(Irq) == 1
			? Busy
			: Ready;
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
