// PN532 library by adafruit/ladyada
// MIT license

// authenticateBlock, readMemoryBlock, writeMemoryBlock contributed
// by Seeed Technology Inc (www.seeedstudio.com)
// 
// backupSPIConf, restoreSPIConf, RFConfiguration, spiread, spiwrite  contributed
// by Javier Montaner (montanerj at yahoo dot com) 2012



#include <Arduino.h>

// Command structure:
// 00 00 FF <LEN> <LCS> D4 <CC> <optional input data> <DCS> 00
// 00 00 FF <LEN> <LCS> D5 <CC+1> <optional output data> <DCS> 00

#define PN532_PREAMBLE    0x00
#define PN532_STARTCODE1  0x00
#define PN532_STARTCODE2  0xFF
#define PN532_POSTAMBLE   0x00

#define PN532_HOSTTOPN532 0xD4

// Miscellanious commands
#define PN532_CMD_DIAGNOSE                 0x00
#define PN532_CMD_GET_FIRMWARE_VERSION     0x02
#define PN532_CMD_GET_GENERAL_STATUS       0x04
#define PN532_CMD_READ_REGISTER            0x06
#define PN532_CMD_WRITE_REGISTER           0x08
#define PN532_CMD_READ_GPIO                0x0C
#define PN532_CMD_WRITE_GPIO               0x0E
#define PN532_CMD_SET_SERIAL_BAUD_RATE     0x10
#define PN532_CMD_SET_PARAMETERS           0x12
#define PN532_CMD_SAM_CONFIGURATION        0x14
#define PN532_CMD_POWER_DOWN               0x16

// RF Communication commands
#define PN532_CMD_RF_CONFIGURATION         0x32
#define PN532_CMD_RF_REGULATION_TEST       0x58

// Initiator commands
#define PN532_CMD_IN_JUMP_FOR_DEP          0x56
#define PN532_CMD_IN_JUMP_FOR_PSL          0x46
#define PN532_CMD_IN_LIST_PASSIVE_TARGET   0x4A
#define PN532_CMD_IN_ATR                   0x50
#define PN532_CMD_IN_PSL                   0x4E
#define PN532_CMD_IN_DATA_EXCHANGE         0x40
#define PN532_CMD_IN_COMMUNICATE_THRU      0x42
#define PN532_CMD_IN_DESELECT              0x44
#define PN532_CMD_IN_RELEASE               0x52
#define PN532_CMD_IN_SELECT                0x54
#define PN532_CMD_IN_AUTO_POLL             0x60

// Target commands
#define PN532_CMD_TG_INIT_AS_TARGET        0x8C
#define PN532_CMD_TG_SET_GENERAL_BYTES     0x92
#define PN532_CMD_TG_GET_DATA              0x86
#define PN532_CMD_TG_SET_DATA              0x8E
#define PN532_CMD_TG_SET_META_DATA         0x94
#define PN532_CMD_TG_GET_INITIATOR_COMMAND 0x88
#define PN532_CMD_TG_RESPONSE_TO_INITIATOR 0x90
#define PN532_CMD_TG_GET_TARGET_STATUS     0x8A


#define PN532_MIFARE_READ 0x30
#define PN532_MIFARE_WRITE 0xA0

#define PN532_AUTH_WITH_KEYA 0x60
#define PN532_AUTH_WITH_KEYB 0x61


#define PN532_WAKEUP 0x55

#define  PN532_SPI_STATREAD 0x02
#define  PN532_SPI_DATAWRITE 0x01
#define  PN532_SPI_DATAREAD 0x03
#define  PN532_SPI_READY 0x01

#define PN532_MIFARE_ISO14443A 0x0
#define PN532_MAX_RETRIES 0x05

#define KEY_A	1
#define KEY_B	2


class PN532 {
public:
    PN532(uint8_t cs);

    void begin();
    void backupSPIConf();
    void restoreSPIConf();	
    void RFConfiguration(uint8_t mxRtyPassiveActivation);
    bool SAMConfig();
    uint32_t getFirmwareVersion();
    uint32_t readPassiveTargetID(uint8_t cardbaudrate);
    bool readPassiveTargetID(uint8_t cardbaudrate, uint8_t* buffer, uint32_t bufferSize);
    template <uint32_t BufferSize>
    bool readPassiveTargetID(uint8_t cardbaudrate, uint8_t(&buffer)[BufferSize])
    {
        return readPassiveTargetID(cardbaudrate, buffer, BufferSize);
    }

    uint32_t authenticateBlock(	uint8_t cardnumber /*1 or 2*/,
                                uint32_t cid /*Card NUID*/,
                                uint8_t blockaddress /*0 to 63*/,
                                uint8_t authtype /*Either KEY_A or KEY_B */,
                                uint8_t const* keys);

    uint32_t readMemoryBlock(uint8_t cardnumber /*1 or 2*/, uint8_t blockaddress /*0 to 63*/, uint8_t* block);
    uint32_t writeMemoryBlock(uint8_t cardnumber /*1 or 2*/, uint8_t blockaddress /*0 to 63*/, uint8_t const* block);

    bool sendCommandCheckAck(uint8_t const* cmd, uint8_t cmdlen, uint16_t timeout = 1000);

    //

private:
    uint8_t const _ss;
    uint8_t _mode, _bitOrder, _spiClock;
    bool spi_readack();
    uint8_t readspistatus();
    void readspidata(uint8_t* buff, uint8_t n);
    void spiwritecommand(uint8_t const* cmd, uint8_t cmdlen);
    void spiwrite(uint8_t c);
    uint8_t spiread();
};
