// PN532 library by adafruit/ladyada
// MIT license

// authenticateBlock, readMemoryBlock, writeMemoryBlock contributed
// by Seeed Technology Inc (www.seeedstudio.com)
// 
// backupSPIConf, restoreSPIConf, RFConfiguration, spiread, spiwrite  contributed
// by Javier Montaner (montanerj at yahoo dot com) 2012



#include <Arduino.h>

// SPI bus policy
#include "SpiBus.h"
// I2C bus policy
//#include "I2cBus.h"

#include "NopDebugPolicy.h"
#include "SerialDebugPolicy.h"


namespace pn532 {

// Command structure:
// 00 00 FF <LEN> <LCS> D4 <CC> <optional input data> <DCS> 00

// Response structure:
// 00 00 FF <LEN> <LCS> D5 <CC+1> <optional output data> <DCS> 00

// XXX: maros
#define PN532_PREAMBLE    0x00
#define PN532_STARTCODE1  0x00
#define PN532_STARTCODE2  0xFF
#define PN532_POSTAMBLE   0x00

/// @see p6.2.1.1, 
enum Tfi
{
    Tfi_HostToPn5xx = 0xD4,
    Tfi_Pn5xxToHost = 0xD5,
};

enum Cmd
{
    // Miscellanious commands
    Cmd_Diagnose                 = 0x00,
    Cmd_GetFirmwareVersion       = 0x02,
    Cmd_GetGeneralStatus         = 0x04,
    Cmd_ReadRegister             = 0x06,
    Cmd_WriteRegister            = 0x08,
    Cmd_ReadGpio                 = 0x0C,
    Cmd_WriteGpio                = 0x0E,
    Cmd_SetSerialBaudRate        = 0x10,
    Cmd_SetParameters            = 0x12,
    Cmd_SamConfiguration         = 0x14,
    Cmd_PowerDown                = 0x16,

    // RF Communication commands
    Cmd_RfConfiguration          = 0x32,
    Cmd_RfRegulationTest         = 0x58,

    // Initiator commands
    Cmd_InJumpForDep             = 0x56,
    Cmd_InJumpForPsl             = 0x46,
    Cmd_InListPassiveTarget      = 0x4A,
    Cmd_InAtr                    = 0x50,
    Cmd_InPsl                    = 0x4E,
    Cmd_InDataExchange           = 0x40,
    Cmd_InCommunicateThru        = 0x42,
    Cmd_InDeselect               = 0x44,
    Cmd_InRelease                = 0x52,
    Cmd_InSelect                 = 0x54,
    Cmd_InAutoPoll               = 0x60,

    // Target commands
    Cmd_TgInitAsTarget           = 0x8C,
    Cmd_TgSetGeneralBytes        = 0x92,
    Cmd_TgGetData                = 0x86,
    Cmd_TgSetData                = 0x8E,
    Cmd_TgSetMetaData            = 0x94,
    Cmd_TgGetInitiatorCommand    = 0x88,
    Cmd_TgResponseToInitiator    = 0x90,
    Cmd_TgGetTargetStatus        = 0x8A
};

/// Calculates a response code from a command.
inline uint8_t responseOf(Cmd cmd) { return cmd+1; }

/// Mifare commands (in Cmd_InDataExchange command DataOut parameter).
enum MifareCmd
{
    MifareCmd_AuthWithKeyA = 0x60,
    MifareCmd_AuthWithKeyB = 0x61,
    MifareCmd_Read16 = 0x30,
    MifareCmd_Write16 = 0xA0,
    MifareCmd_Write4 = 0xA2,
    MifareCmd_Increment = 0xC1,
    MifareCmd_Decrement = 0xC0,
    MifareCmd_Transfer = 0xB0,
    MifareCmd_Restore = 0xC2
};

#define PN532_WAKEUP 0x55

// See datasheet p6.2.5
#define  PN532_SPI_STATREAD  0x02
#define  PN532_SPI_DATAWRITE 0x01
#define  PN532_SPI_DATAREAD  0x03

#define  PN532_SPI_READY 0x01

//#define PN532_MIFARE_ISO14443A 0x0
//#define PN532_MAX_RETRIES      0x05


enum Key
{
    Key_A = 1,
    Key_B = 2
};

enum SamMode// : uint8_t
{
    SamMode_Normal = 0x01,
    SamMode_VirtualCard = 0x02,
    SamMode_WiredCard = 0x03,
    SamMode_DualCard = 0x04,
};

enum SamIrq// : uint8_t
{
    SamIrq_No = 0x00,
    SamIrq_Yes = 0x01,
};

/// @see datasheet p. 7.3.5
enum BrTy
{
    BrTy_106kbpsTypeA = 0x00,
    BrTy_212kbpsFeliCa = 0x01,
    BrTy_424kbpsFeliCa = 0x02,
    BrTy_106kbpsTypeB = 0x03,
    BrTy_106kbpsJewel = 0x04
};

/// @see dataseet p.7.3.1
enum RfConfigurationItem
{
    RfConfigurationItem_RfField = 0x01,
    RfConfigurationItem_VariousTimings = 0x02,
    RfConfigurationItem_MaxRtyCom = 0x04,
    RfConfigurationItem_MaxRetries = 0x05,
    RfConfigurationItem_AnalogSettings_for_106kbpsTypeA = 0x0A,
    RfConfigurationItem_AnalogSettings_for_212_424kbps = 0x0B,
    RfConfigurationItem_AnalogSettings_for_TypeB = 0x0C,
    RfConfigurationItem_AnalogSettings_for_212_424_848 = 0x0D
};

/// @see p7.1
enum Error
{
    Error_Timeout = 0x01,
    Error_Crc     = 0x02,
    Error_Parity  = 0x03,
    // ...
    Error_DepProtocol = 0x13
    // ...
};

// XXX: these structs seem like a stupid idea. revert to putting the command together in the functions.

struct GetFirmwareVersion
{
    static uint8_t const Code = Cmd_GetFirmwareVersion;
    static uint8_t const ResponseCode = Code+1;
    explicit GetFirmwareVersion(uint8_t* buf);
    uint8_t length() const { return 1; }
    uint8_t const* const ptr;
};

struct RfConfiguration
{
    static uint8_t const Code = Cmd_RfConfiguration;
    static uint8_t const ResponseCode = Code+1;
    RfConfiguration(uint8_t* buf, RfConfigurationItem cfgItem, void const* cfgData);
    uint8_t length() const { return length_; }
    uint8_t const* const ptr;
private:
    uint8_t length_;
};

/// Secure Access Module conf.
struct SamConfiguration
{
    static uint8_t const Code = Cmd_SamConfiguration;
    static uint8_t const ResponseCode = Code+1;
    /// @param timeout_50ms - timeout in 50ms units.
    SamConfiguration(uint8_t* buf, SamMode samMode, uint8_t timeout_50ms, SamIrq samIrq = SamIrq_Yes);
    uint8_t length() const { return 4; }
    uint8_t const* const ptr;
};

struct InListPassiveTarget
{
    static uint8_t const Code = Cmd_InListPassiveTarget;
    static uint8_t const ResponseCode = Code+1;
    InListPassiveTarget(uint8_t* buf, uint8_t maxTags, BrTy brTy, uint8_t const* initiatorData);
    uint8_t length() const { return 3; }
    uint8_t const* const ptr;
    //private:
    //    uint8_t length_;
};

struct InDataExchange
{
    static uint8_t const Code = Cmd_InDataExchange;
    static uint8_t const ResponseCode = Code+1;
    InDataExchange();
    uint8_t length() const;
    uint8_t const* const ptr;
};

// The aim of the refactor is to allow user of this library to do this:
//
// typedef pn532::PN532<SpiBus<1>, NopDebug> NFC;

struct PN532Base_
{
    static uint8_t const PacketBufferSize = 64;
    static uint8_t packetBuffer[PacketBufferSize];
};

// TODO: rename this class!
template <typename BusPolicy, typename DebugPolicy>
class PN532Base : public PN532Base_
{
public:

    /// This is a type for return values of the functions in this class. Just plain success/fail is
    /// not enough - we need a reason of failure.
    enum ReturnValue
    {
        Success,
        S_AckFrame,
        S_NackFrame,
        E_Fail,
        E_InsufficientBuffer,
        E_MultipleTags,
        E_DataChecksum,
        E_LengthChecksum,
        E_ApplicationError,
        E_ErrorFrame,
    };

    typedef BusPolicy Bus;
    typedef DebugPolicy Debug;

    static void begin();

    //
    // Generic stuff
    //

    static void setRfMaxRetries(uint8_t mxRtyPassiveActivation);
    static bool SAMConfig();
    static uint32_t getFirmwareVersion();
    static bool sendCommandCheckAck(uint8_t const* cmd, uint8_t cmdlen, uint16_t timeout);
    template <typename Command>
    static bool sendCommandCheckAck(Command const& cmd, uint16_t timeout = 1000)
    {
        return sendCommandCheckAck(cmd.ptr, cmd.length(), timeout);
    }

    //
    // ISO 14443-3A
    //

    /// Read id of a passive target.
    /// @param cardbaudrate
    /// @param buffer - pointer to a buffer that can hold at least \a bufferSize bytes.
    /// @param bufferSize - number of bytes that the buffer can hold.
    /// @param idLength - number of bytes that the ID read from the target contains.
    /// @retval Success if the reading is successful.
    /// @retval E_InsufficientBuffer if the bufferSize is too small to fit entire id into it.
    /// @retval E_Fail - is reading the id failed.
    static ReturnValue readPassiveTargetID(BrTy cardbaudrate, uint8_t* buffer, uint32_t bufferSize, uint8_t& idLength);
    template <uint8_t BufferSize>
    static ReturnValue readPassiveTargetID(BrTy cardbaudrate, uint8_t(&buffer)[BufferSize], uint8_t& idLength)
    {
        return readPassiveTargetID(cardbaudrate, buffer, BufferSize, idLength);
    }

    //
    // Mifare classic
    //

    static uint32_t authenticateBlock(
        uint8_t cardnumber /*1 or 2*/,
        uint8_t const* id, uint8_t idLength,
        uint8_t blockaddress /*0 to 63*/,
        Key authtype,
        uint8_t const* keys);

    static bool readMemoryBlock(
        uint8_t cardnumber /*1 or 2*/,
        uint8_t blockaddress /*0 to 63*/,
        uint8_t* block);

    static bool writeMemoryBlock(
        uint8_t cardnumber /*1 or 2*/,
        uint8_t blockaddress /*0 to 63*/,
        uint8_t const* block);

    // format ndef
    // write ndef

    //
    // Mifare Ultralight
    //

    // ...

    //
    // Desfire
    //

    // ...

    static ReturnValue readData(uint8_t* buff, uint8_t n);
    template <uint8_t Size>
    static ReturnValue readData(uint8_t (&buff)[Size])
    {
        return readData(buff, Size);
    }

private:
    static bool hasAck();
    static bool waitReady(uint16_t timeout);
    static uint8_t readStatus();
    static void writeCommand(uint8_t const* cmd, uint8_t cmdlen);
    //static bool isErrorFrame(uint8_t const* frame, uint8_t frameLen);
};

template <typename BusPolicy, typename DebugPolicy>
struct Iso14443_3A : public PN532Base<BusPolicy, DebugPolicy>
{
    // functions that you can do with ISO-14443-3A.
};

template <typename BusPolicy, typename DebugPolicy>
struct MifareClassic : public PN532Base<BusPolicy, DebugPolicy>
{
    // functions that you can do with Mifare Classic.
};

template <typename BusPolicy, typename DebugPolicy>
struct MifareUltralight : public PN532Base<BusPolicy, DebugPolicy>
{
    // functions that you can do with Mifare Ultralight.
};

template <typename BusPolicy, typename DebugPolicy>
struct Desfire : public PN532Base<BusPolicy, DebugPolicy>
{
    // functions that you can do with Desfire
};


static byte const pn532ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
static byte const pn532response_firmwarevers[] = {0x00, 0xFF, 0x06, 0xFA, 0xD5, 0x03};

template <typename B, typename D>
void PN532Base<B,D>::begin()
{
    Bus::begin();
    Bus::select();

    delay(1000);

    // not exactly sure why but we have to send a dummy command to get synced up
    sendCommandCheckAck(GetFirmwareVersion(packetBuffer));

    // ignore response!
}

template <typename B, typename D>
uint32_t PN532Base<B,D>::getFirmwareVersion()
{
    if (! sendCommandCheckAck(GetFirmwareVersion(packetBuffer)))
    {
        Debug::println("send command failed");
        return 0;
    }

    // read data packet
    readData(packetBuffer, 12);

    // check some basic stuff
    // 0x00, 0xFF, 0x06, 0xFA, 0xD5, 0x03
    for (uint8_t i = 0; i < 12; ++i)
    {
        Debug::print(packetBuffer[i], HEX); Debug::print(" ");
    }
    if (0 != memcmp(packetBuffer+1, pn532response_firmwarevers, 6))
    {
        Debug::println("response compare failed");
        return 0;
    }

    uint32_t response = 0;
    for (uint8_t const* p = packetBuffer+6; p < packetBuffer+10; ++p)
    {
        response <<= 8;
        response |= *p;
    }

    return response;
}

// Define a timeout to stop looking for a tag/card e.g. in 
// This function avoids to block arduino flow at readPassiveTargetID() method if no tag/card is detected
//
// From PN532 manual:
//    MxRtyPassiveActivation is a byte containing the number of times that the
//    PN532 will retry to activate a target in InListPassiveTarget command
//    Value 0xFF means to try eternally, 0x00 means only once (no retry, only one try).
//    The default value of this parameter is 0xFF (infinitely).
template <typename B, typename D>
void PN532Base<B,D>::setRfMaxRetries(uint8_t mxRtyPassiveActivation)
{
    uint8_t const cfgData[3] =
    {
        0xFF, // default MxRtyATR
        0x01, // default MxRtyPSL
        mxRtyPassiveActivation
    };

    sendCommandCheckAck(RfConfiguration(packetBuffer, RfConfigurationItem_MaxRetries, cfgData));
    // ignore response!
}

template <typename B, typename D>
bool PN532Base<B,D>::waitReady(uint16_t const timeout)
{
    uint16_t timer = 0;
    // Wait for chip to say its ready!
    while (readStatus() != PN532_SPI_READY)
    {
        if (timeout != 0)
        {
            timer += 10;
            if (timer > timeout)
            {
                return false;
            }
        }
        delay(10);
    }

    return true;
}

// default timeout of one second
template <typename B, typename D>
bool PN532Base<B,D>::sendCommandCheckAck(uint8_t const* cmd, uint8_t cmdlen, uint16_t timeout)
{
    // write the command
    writeCommand(cmd, cmdlen);

    return waitReady(timeout)
        && PN532Base::hasAck()
        && waitReady(timeout);
}

template <typename B, typename D>
bool PN532Base<B,D>::SAMConfig()
{
    if (! sendCommandCheckAck(SamConfiguration(packetBuffer, SamMode_Normal, 20, SamIrq_Yes)))
        return false;

    // read data packet
    readData(packetBuffer, 8);

    // XXX: magic number 5
    return packetBuffer[5] == SamConfiguration::ResponseCode;
}

template <typename B, typename D>
uint32_t PN532Base<B,D>::authenticateBlock(
    uint8_t cardnumber /*1 or 2*/,
    uint8_t const* id, uint8_t idLength/*Card NUID*/,
    uint8_t blockaddress /*0 to 63*/,
    Key authtype,
    uint8_t const* keys)
{
    uint8_t* p = packetBuffer;
    *(p++) = Cmd_InDataExchange;
    *(p++) = cardnumber;  // either card 1 or 2 (tested for card 1)
    if (authtype == Key_A)
    {
        *(p++) = MifareCmd_AuthWithKeyA;
    }
    else
    {
        *(p++) = MifareCmd_AuthWithKeyB;
    }
    *(p++) = blockaddress; //This address can be 0-63 for MIFARE 1K card

    memcpy(p, keys, 6);
    p += 6;

    memcpy(packetBuffer+10, id, idLength);
    p += idLength;

    if (! sendCommandCheckAck(packetBuffer, p - packetBuffer, 1000))
        return false;

    // read data packet
    readData(packetBuffer, 2+6);

    for (int iter = 0; iter < 14; ++iter)
    {
        Debug::print(packetBuffer[iter], HEX);
        Debug::print(" ");
    }
    Debug::println();
    // check some basic stuff

    Debug::println("AUTH");
    for (uint8_t i = 0; i < 2+6; ++i)
    {
        Debug::print(packetBuffer[i], HEX); Debug::println(" ");
    }

    // XXX: magic numbers.
    return packetBuffer[6] == responseOf(Cmd_InDataExchange) && packetBuffer[7] == 0x00;
}

template <typename B, typename D>
bool PN532Base<B,D>::readMemoryBlock(uint8_t cardnumber /*1 or 2*/,uint8_t blockaddress /*0 to 63*/, uint8_t* block) {
    packetBuffer[0] = Cmd_InDataExchange;
    packetBuffer[1] = cardnumber;  // either card 1 or 2 (tested for card 1)
    packetBuffer[2] = MifareCmd_Read16;
    packetBuffer[3] = blockaddress; //This address can be 0-63 for MIFARE 1K card

    if (!sendCommandCheckAck(packetBuffer, 4, 1000))
        return false;

    // read data packet
    readData(packetBuffer, 18+6);

    Debug::print("READ: ");
    for(uint8_t i=8;i<18+6;i++)
    {
        block[i-8] = packetBuffer[i];
        Debug::print(packetBuffer[i], HEX); Debug::print(" ");
    }
    Debug::print("\nStatus: 0x"); Debug::println(packetBuffer[7], HEX);

    // XXX: Magic numbers
    return (packetBuffer[6] == responseOf(Cmd_InDataExchange)) && (packetBuffer[7] == 0x00);
}

//Do not write to Sector Trailer Block unless you know what you are doing.
template <typename B, typename D>
bool PN532Base<B,D>::writeMemoryBlock(uint8_t cardnumber /*1 or 2*/,uint8_t blockaddress /*0 to 63*/, uint8_t const* block) {
    packetBuffer[0] = Cmd_InDataExchange;
    packetBuffer[1] = cardnumber;  // either card 1 or 2 (tested for card 1)
    packetBuffer[2] = MifareCmd_Write16;
    packetBuffer[3] = blockaddress;

    for(uint8_t byte=0; byte <16; byte++)
    {
        packetBuffer[4+byte] = block[byte];
    }

    if (! sendCommandCheckAck(packetBuffer, 20, 1000))
        return false;
    // read data packet
    readData(packetBuffer, 2+6);

    // check some basic stuff
    Debug::println("WRITE");
    for (uint8_t i = 0; i < 2+6; ++i)
    {
        Debug::print(packetBuffer[i], HEX); Debug::println(" ");
    }

    // XXX: magic numbers
    return (packetBuffer[6] == responseOf(Cmd_InDataExchange)) && (packetBuffer[7] == 0x00);
}

template <typename B, typename D>
typename PN532Base<B,D>::ReturnValue PN532Base<B,D>::readPassiveTargetID(BrTy cardbaudrate, uint8_t* buffer, uint32_t bufferSize, uint8_t& idLength)
{
    if (! sendCommandCheckAck(InListPassiveTarget(packetBuffer, 1, cardbaudrate, NULL)))
    {
        return E_Fail;  // no cards read
    }

    // read data packet
    readData(packetBuffer, 20);

    uint8_t const responseCode = packetBuffer[6]; // XXX: magic number
    if (responseCode != responseOf(Cmd_InListPassiveTarget))
    {
        Debug::print("Error: 0x"); Debug::print(responseCode, HEX);
        return E_Fail;
    }
    else
    {
        uint8_t const numberOfTags = packetBuffer[7]; // XXX: magic number
        Debug::print("Found "); Debug::print(numberOfTags); Debug::println(" tags");

        if (numberOfTags != 1)
        {
            return E_MultipleTags;
        }

        uint16_t sens_res = packetBuffer[9];
        sens_res <<= 8;
        sens_res |= packetBuffer[10];
        Debug::print("Sens Response: 0x");  Debug::println(sens_res, HEX);
        Debug::print("Sel Response : 0x");  Debug::println(packetBuffer[11], HEX);

        idLength = packetBuffer[12];
        for (uint8_t i = 0; i < idLength && i < bufferSize; ++i)
        {
            buffer[i] = packetBuffer[12+i];
        }

        Debug::print("TargetID: ");
        for (uint8_t i = 0; i < 20; ++i)
        {
            uint8_t const b = packetBuffer[i];
            Debug::print(b/16, HEX); Debug::print(b%16, HEX); Debug::print(" ");
        }
        Debug::println();

        return Success;
    }
}


/************** high level SPI */


template <typename B, typename D>
bool PN532Base<B,D>::hasAck()
{
    uint8_t ackbuff[7];
    ReturnValue const res = readData(ackbuff);
    return res == S_AckFrame;
}

/************** mid level SPI */
//Send PN532_SPI_STATREAD byte to check the PN532 status 
//PN532 only answers PN532_SPI_READY once it has processed 
//the previous command either to send an ACK or to send the 
//final response to the command
template <typename B, typename D>
uint8_t PN532Base<B,D>::readStatus()
{
    Bus::select();
    delay(2);
    uint8_t const x = Bus::transceiveByte(PN532_SPI_STATREAD);   
    Bus::deselect();
    return x;
}

/// Reading frame from PN532.
///
/// PN532 only answers PN532_SPI_READY once it has processed 
/// the previous command either to send an ACK or to send the 
/// final response to the command
///
template <typename B, typename D>
typename PN532Base<B,D>::ReturnValue PN532Base<B,D>::readData(uint8_t* const buff, uint8_t n)
{
    Debug::print("\nreadData"); Debug::print("buflen="); Debug::println(n);
    // 
    // TODO: move this to the Bus.
    struct ScopedSelect {
        ScopedSelect() { Bus::select(); }
        ~ScopedSelect() { Bus::deselect(); }
    } _;

    delay(2);
    Bus::dataReadFollows(n+2); //read leading byte DR and discard

    // 00 00 FF <LEN> <LCS> D5 <CC+1> <optional output data> <DCS> 00
    enum ReadState {
        ReadingPreamble,
        ReadingStartCode1,
        ReadingStartCode2,
        ReadingLength,
        ReadingLcs,
        ReadingTfi,
        ReadingOutputData,
        ReadingDcs,
        ReadingPostamble,
        Done
    };

    ReadState state = ReadingPreamble;
    uint8_t length = 0;
    uint8_t* p = buff;
    uint8_t* end = buff+n;
    uint8_t dcs = 0xd5;
    while (state != Done)
    {
        if (p > end)
        {
            Debug::println("\nError: Insufficient buffer");
            Debug::print("state="); Debug::println(uint8_t(state));
            return E_InsufficientBuffer;
        }
        uint8_t const b = *(p++) = Bus::readByte();
        Debug::print(b / 16, HEX); Debug::print(b%16, HEX); Debug::print(" ");

        switch (state)
        {
        case ReadingPreamble:
            if (b == 0x00)
                state = ReadingStartCode1;
            break;
        case ReadingStartCode1:
            if (b == 0x00)
            {
                state = ReadingStartCode2;
            }
            else
            {
                p = buff;
                Debug::println();
            }
            break;
        case ReadingStartCode2:
            if (b == 0xff)
            {
                state = ReadingLength;
            }
            else if (b == 0x00)
            {
                state = ReadingStartCode2;
                p = buff+1;
            }
            else
            {
                state = ReadingStartCode1;
                p = buff;
                Debug::println();
            }
            break;
        case ReadingLength:
                length = b;
                state = ReadingLcs;
                break;
        case ReadingLcs:
            if (b == 0xff && length == 0)
            {
                *(p++) = Bus::readByte();
                return S_AckFrame;
            }
            else if (b == 0 && length == 0xff)
            {
                *(p++) = Bus::readByte();
                return S_NackFrame;
            }
            else if (uint8_t(b+length) != 0)
            {
                Debug::println("Error: length checksum mismatch");
                return E_LengthChecksum;
            }
            else
            {
                state = ReadingTfi;
            }
            break;
        case ReadingTfi:
            if (b != Tfi_Pn5xxToHost)
                return E_ErrorFrame;
            else
            {
                state = ReadingOutputData;
                --length; // TFI is included in length
            }
            break;
        case ReadingOutputData:
            if (length > 0)
            {
                dcs += b;
                --length;
                break;
            }
            else
            {
                state = ReadingDcs;
            }
            // no break
        case ReadingDcs:
            if (uint8_t(dcs + b) != 0)
            {
                Debug::println("Error: data checksum error");
                return E_DataChecksum;
            }
            else
            {
                state = ReadingPostamble;
            }
            break;
        case ReadingPostamble:
            /*if (b != 0)
            {
                Debug::println("Error: Invalid postamble");
                return E_InvalidPostamble;
            }*/
            state = Done;
            break;
        }
    }

    Debug::println();

    return Success;
}

template <typename B, typename D>
void PN532Base<B,D>::writeCommand(uint8_t const* cmd, uint8_t cmdlen)
{
    ++cmdlen;

    Debug::print("\nSending: ");

    Bus::select();
    delay(2);     // or whatever the delay is for waking up the board
    Bus::beginTransmission();

    uint8_t checksum = PN532_PREAMBLE + PN532_STARTCODE1 + PN532_STARTCODE2;
    Bus::writeByte(PN532_PREAMBLE);
    Bus::writeByte(PN532_STARTCODE1);
    Bus::writeByte(PN532_STARTCODE2);

    Bus::writeByte(cmdlen);
    uint8_t const cmdlen_1 = ~cmdlen + 1;
    Bus::writeByte(cmdlen_1);

    Bus::writeByte(Tfi_HostToPn5xx);
    checksum += Tfi_HostToPn5xx;

    Debug::print(" 0x"); Debug::print(Tfi_HostToPn5xx, HEX);
    for (uint8_t i=0; i<cmdlen-1; i++)
    {
        Bus::writeByte(cmd[i]);
        checksum += cmd[i];
    }
    uint8_t const checksum_1 = ~checksum;
    Bus::writeByte(checksum_1);
    Bus::writeByte(PN532_POSTAMBLE);
    Bus::endTransmission();
    Bus::deselect();

    Debug::print(" 0x"); Debug::print(PN532_PREAMBLE, HEX);
    Debug::print(" 0x"); Debug::print(PN532_PREAMBLE, HEX);
    Debug::print(" 0x"); Debug::print(PN532_STARTCODE2, HEX);
    Debug::print(" 0x"); Debug::print(cmdlen, HEX);
    Debug::print(" 0x"); Debug::print(cmdlen_1, HEX);
    Debug::print(" 0x"); Debug::print(Tfi_HostToPn5xx, HEX);

    for (uint8_t i=0; i<cmdlen-1; i++)
    {
        Debug::print(" 0x"); Debug::print(cmd[i], HEX);
    }
    Debug::print(" 0x"); Debug::print(checksum_1, HEX);
    Debug::print(" 0x"); Debug::print(PN532_POSTAMBLE, HEX);
    Debug::println();
} 

//template <typename B, typename D>
//bool PN532Base<B,D>::isErrorFrame(uint8_t const* frame, uint8_t frameLen)
//{
//    // 0x00 0xFF 0x01 0xFF 0x7F 0x81 
//    // |    |    |    |     |    | 
//    // |    |    |    |     |    +------- packet data checksum
//    // |    |    |    |     +------------ specific application level error code
//    // |    |    |    +------------------ packet length error code
//    // |    |    +----------------------- packet length checksum
//    // +----+---------------------------- start of packet
//    
//}

} // namespace pn532
