#pragma once

#include <avr/io.h>

#if !defined(_BV)
#  define _BV(x) (1 << (x))
#endif

namespace pn532
{

#define DECLARE_PORT(x) \
struct impl_Port_ ## x { \
    static void makeOutput(uint8_t mask) { DDR ## x |= mask; } \
    static void makeInput(uint8_t mask)  { DDR ## x &= ~mask; } \
    static void setBits(uint8_t mask)    { PORT ## x |= mask; } \
    static uint8_t getBits()             { return PORT ## x; } \
    static void clearBits(uint8_t mask)  { PORT ## x &= ~mask; } \
}

#if defined(__AVR_ATmega328P__)
struct AvrDevice
{
    typedef DECLARE_PORT(B) PortB;
    typedef DECLARE_PORT(C) PortC;
    typedef DECLARE_PORT(D) PortD;

    typedef PortB SpiPort;
    static uint8_t const SSPin = PORTB2;
    static uint8_t const SCKPin = PORTB5;
    static uint8_t const MOSIPin = PORTB3;
    static uint8_t const MISOPin = PORTB4;
};
#elif defined(__AVR_ATmega2560__)
struct AvrDevice
{
    typedef DECLARE_PORT(A) PortA;
    typedef DECLARE_PORT(B) PortB;
    typedef DECLARE_PORT(C) PortC;
    typedef DECLARE_PORT(D) PortD;
    typedef DECLARE_PORT(E) PortE;
    typedef DECLARE_PORT(F) PortF;
    typedef DECLARE_PORT(G) PortG;
    typedef DECLARE_PORT(H) PortH;
    typedef DECLARE_PORT(J) PortJ;
    typedef DECLARE_PORT(K) PortK;
    typedef DECLARE_PORT(L) PortL;

    typedef PortB SpiPort;
    static uint8_t const SSPin   = PORTB0;
    static uint8_t const SCKPin  = PORTB1;
    static uint8_t const MOSIPin = PORTB2;
    static uint8_t const MISOPin = PORTB3;
};
#else
#  error unsupported cpu
#endif

template <typename PortType, uint8_t PN>
struct Pin
{
    static uint8_t const PinNumber = PN;
    typedef PortType Port;

    static void    high() { Port::setBits(_BV(PinNumber)); }
    static uint8_t get()  { return Port::getBits() >> PinNumber; }
    static void    low()  { Port::clearBits(_BV(PinNumber)); }

    static void makeOutput() { Port::makeOutput(_BV(PinNumber)); }
    static void makeInput()  { Port::makeInput(_BV(PinNumber));  }
};


template <
        typename PortType = AvrDevice::SpiPort,
        uint8_t SCKPin  = AvrDevice::SCKPin,
        uint8_t MOSIPin = AvrDevice::MOSIPin,
        uint8_t MISOPin = AvrDevice::MISOPin,
        uint8_t SSPin   = AvrDevice::SSPin,
        bool HighSelects = false>
struct AvrSpiBus
{
    typedef PortType Port;
    typedef Pin<Port, SSPin> SlaveSelectPin;
    typedef Pin<Port, SCKPin> ClockPin;
    typedef Pin<Port, MOSIPin> MosiPin;
    typedef Pin<Port, MISOPin> MisoPin;

    // XXX: NFC dev
    typedef Pin<Port, PORTB4> NfcSelectPin;

    enum { Busy, Ready };

    static void begin()
    {
        uint8_t const oldSREG = SREG;
        cli();

        // Set direction register for SCK and MOSI pin.
        // MISO pin automatically overrides to INPUT.
        // When the SS pin is set as OUTPUT, it can be used as
        // a general purpose output port (it doesn't influence
        // SPI operations).
        ClockPin::makeOutput();
        MosiPin::makeOutput();
        MisoPin::makeInput();
        SlaveSelectPin::makeOutput();
        NfcSelectPin::makeOutput();

        ClockPin::low();
        MosiPin::low();
        SlaveSelectPin::high();
        NfcSelectPin::high();

        SPCR |= _BV(MSTR);
        SPCR |= _BV(SPE);

        SREG = oldSREG;

        // Warning: if the SS pin ever becomes a LOW INPUT then SPI
        // automatically switches to Slave, so the data direction of
        // the SS pin MUST be kept as OUTPUT.
    }

    static void end()
    {
        SPCR &= _BV(SPE);
    }

    static void select()
    {
        if (HighSelects)
            NfcSelectPin::high();
        else
            NfcSelectPin::low();
    }

    static void deselect()
    {
        if (HighSelects)
            NfcSelectPin::low();
        else
            NfcSelectPin::high();
    }

    static uint8_t transceiveByte(uint8_t data)
    {
        SPDR = data;
        while (!(SPSR & _BV(SPIF)))
        { }
        return SPDR;
    }

    enum OperationType
    {
        OperationType_StatusReading = 0x02,
        OperationType_DataWriting   = 0x01,
        OperationType_DataReading   = 0x03
    };

    static uint8_t readByte()
    {
        return transceiveByte(OperationType_DataReading);
    }

    static void writeByte(uint8_t b)
    {
        transceiveByte(b);
    }

    static uint8_t beginTransmission()
    {
        return transceiveByte(OperationType_DataWriting);
    }

    /// Used by I2C.
    static void endTransmission() { /* NOP */ }
    static uint8_t dataReadFollows(uint8_t /*bytesToRead*/)
    {
        return readByte();
    }

    static uint8_t readStatus()
    {
        return transceiveByte(OperationType_StatusReading);
    }

    enum BitOrder
    {
        BitOrder_LsbFirst,
        BitOrder_MsbFirst
    };

    enum SpiClock
    {
        SpiClock_DIV4 = 0x00,
        SpiClock_DIV16 = 0x01,
        SpiClock_DIV64 = 0x02,
        SpiClock_DIV128 = 0x03,
        SpiClock_DIV2 = 0x04,
        SpiClock_DIV8 = 0x05,
        SpiClock_DIV32 = 0x06//,
        //SpiClock_DIV64 = 0x07
    };

    enum SpiMode
    {
        SpiMode_0 = 0x00,
        SpiMode_1 = 0x04,
        SpiMode_2 = 0x08,
        SpiMode_3 = 0x0C,
        SpiMode_MASK = 0x0C  // CPOL = bit 3, CPHA = bit 2 on SPCR
    };

    static uint8_t const SpiClock_MASK = 0x03;  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
    static uint8_t const Spi2XClock_MASK = 0x01;  // SPI2X = bit 0 on SPSR

    static void backupConfiguration()
    {
        mode = SPCR & SpiMode_MASK;
        bitOrder =  SPCR & _BV(DORD);
        spiClock = SPCR & SpiClock_MASK;
        setDataMode(SpiMode_0);
        setBitOrder(BitOrder_LsbFirst);
        setClockDivider(SpiClock_DIV4);
    }

    static void restoreConfiguration()
    {
        setDataMode(mode);
        if (bitOrder)
            SPCR|=_BV(DORD);
        else
            SPCR &= ~(_BV(DORD));
        setClockDivider(spiClock);
    }

private:

    // Backup configuration
    static uint8_t mode;
    static uint8_t bitOrder;
    static uint8_t spiClock;

    static void setDataMode(uint8_t mode)
    {
        SPCR = (SPCR & ~SpiMode_MASK) | mode;
    }

    static void setClockDivider(uint8_t rate)
    {
        SPCR = (SPCR & ~SpiClock_MASK) | (rate & SpiClock_MASK);
        SPSR = (SPSR & ~Spi2XClock_MASK) | ((rate >> 2) & Spi2XClock_MASK);
    }

    static void setBitOrder(BitOrder bitOrder)
    {
        if (bitOrder == BitOrder_LsbFirst)
        {
            SPCR |= _BV(DORD);
        }
        else
        {
            SPCR &= ~(_BV(DORD));
        }
    }
};

template <typename T,uint8_t a,uint8_t b,uint8_t c,uint8_t d,bool e>
uint8_t AvrSpiBus<T,a,b,c,d,e>::mode = 0;

template <typename T,uint8_t a,uint8_t b,uint8_t c,uint8_t d,bool e>
uint8_t AvrSpiBus<T,a,b,c,d,e>::bitOrder = 0;

template <typename T,uint8_t a,uint8_t b,uint8_t c,uint8_t d,bool e>
uint8_t AvrSpiBus<T,a,b,c,d,e>::spiClock = 0;



} // pn532
