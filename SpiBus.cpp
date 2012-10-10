#include "SpiBus.h"

#include <PN532.h>
#include <SPI.h>

using pn532::SpiBusBase;

static uint8_t mode; 
static uint8_t bitOrder;
static uint8_t spiClock;

void SpiBusBase::begin()
{
	SPI.begin();
}

// Backup SPI values in SPI SPCR register (mode, bit order and spi speed) and set the values required for PN532
// Also select PN532 by setting select PIN to LOW
// Note for example EthernetShield uses MSBFIRST while PN532 uses LSBFIRST
// This method  must be called EVERY TIME before calling  any other method in this library
void SpiBusBase::backupConfiguration() {
	mode = SPCR & SPI_MODE_MASK;
	bitOrder =  SPCR & _BV(DORD);
	spiClock = SPCR & SPI_CLOCK_MASK;
	SPI.setDataMode(SPI_MODE0);
	SPI.setBitOrder(LSBFIRST);
	SPI.setClockDivider(SPI_CLOCK_DIV4);
};

// Restore SPI values in SPI SPCR register (mode, bit order and spi speed) to the values set up by other libraries/shields 
// Also deselect PN532 by setting select PIN to HIGH
// This method  must be called EVERY TIME after calling any other method (or set of methods) in this library
void SpiBusBase::restoreConfiguration() {
	SPI.setDataMode(mode);
	if (bitOrder) SPCR|=_BV(DORD);
	else  SPCR &= ~(_BV(DORD));
	SPI.setClockDivider(spiClock);
};

/************** low level SPI */
//Use official SPI HW library of arduino

// Use official HW write function (register based)
void SpiBusBase::writeByte(uint8_t b)
{
	SPI.transfer(b);
}
	
// Use official HW read function (register based)
// Following PN532 Manual, it writes one byte (PN532_SPI_DATAREAD) and 
// waits for the response byte from PN532
// PN532 only sends data back if it has responded first with
// byte PN532_SPI_READY to a previous command
uint8_t SpiBusBase::readByte()
{
	return SPI.transfer(PN532_SPI_DATAREAD);
}

uint8_t SpiBusBase::transceiveByte(uint8_t b)
{
	return SPI.transfer(b);
}

uint8_t SpiBusBase::dataWriteFollows()
{
    return SpiBusBase::transceiveByte(PN532_SPI_DATAWRITE);
}

uint8_t SpiBusBase::dataReadFollows()
{
    return SpiBusBase::transceiveByte(PN532_SPI_DATAREAD);
}

//uint8_t SpiBusBase::statusReadFollows()
//{
//    return SpiBusBase::transceiveByte(PN532_SPI_STATREAD);
//}
