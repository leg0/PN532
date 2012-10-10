#pragma once

#include <Arduino.h>

namespace pn532
{

struct SpiBus
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

	static uint8_t const ChipSelect = 10;//PN532_CS;

private:
	static uint8_t mode; 
	static uint8_t bitOrder;
	static uint8_t spiClock;
};

} // pn532
