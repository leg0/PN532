// PN532 library by adafruit/ladyada
// MIT license

// authenticateBlock, readMemoryBlock, writeMemoryBlock contributed
// by Seeed Technology Inc (www.seeedstudio.com
//
// backupSPIConf, restoreSPIConf, RFConfiguration, spiread, spiwrite contributed
// by Javier Montaner (montanerj at yahoo dot com) 2012

#include <Arduino.h>
#include <PN532.h>
#include <SPI.h>

static byte const pn532ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
static byte const pn532response_firmwarevers[] = {0x00, 0xFF, 0x06, 0xFA, 0xD5, 0x03};

#define PN532_PACKBUFFSIZ 64
static byte pn532_packetbuffer[PN532_PACKBUFFSIZ];

PN532::CmdGetFirmwareVersion::CmdGetFirmwareVersion(uint8_t* buf)
    : ptr(buf)
{
    buf[0] = PN532_CMD_GET_FIRMWARE_VERSION;
}

PN532::CmdSamConfiguration::CmdSamConfiguration(uint8_t* buf, SamMode samMode, uint8_t timeout_50ms, SamIrq samIrq)
    : ptr(buf)
{
    *(buf++) = PN532_CMD_SAM_CONFIGURATION;
    *(buf++) = samMode;
    *(buf++) = timeout_50ms;
    *(buf++) = samIrq;
}

PN532::CmdInListPassiveTarget::CmdInListPassiveTarget(uint8_t* buf, uint8_t maxTags, BrTy brTy, uint8_t const* /*initiatorData*/)
    : ptr(buf)
{
    *(buf++) = PN532_CMD_IN_LIST_PASSIVE_TARGET;
    *(buf++) = maxTags;
    *(buf++) = brTy;

    // TODO:initiator data.
}


void PN532::begin()
{
    Bus::begin();
    Bus::select();

    delay(1000);

    // not exactly sure why but we have to send a dummy command to get synced up
    sendCommandCheckAck(CmdGetFirmwareVersion(pn532_packetbuffer));

    // ignore response!
}

uint32_t PN532::getFirmwareVersion()
{
    if (! sendCommandCheckAck(CmdGetFirmwareVersion(pn532_packetbuffer)))
    {
        return 0;
    }

    // read data packet
    readData(pn532_packetbuffer, 12);

    // check some basic stuff
    if (0 != memcmp(pn532_packetbuffer, pn532response_firmwarevers, 6))
    {
        return 0;
    }

    uint32_t response = pn532_packetbuffer[6];
    response <<= 8;
    response |= pn532_packetbuffer[7];
    response <<= 8;
    response |= pn532_packetbuffer[8];
    response <<= 8;
    response |= pn532_packetbuffer[9];

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

void PN532::RFConfiguration(uint8_t mxRtyPassiveActivation) {
    pn532_packetbuffer[0] = PN532_CMD_RF_CONFIGURATION;
    pn532_packetbuffer[1] = PN532_MAX_RETRIES; 
    pn532_packetbuffer[2] = 0xFF; // default MxRtyATR
    pn532_packetbuffer[3] = 0x01; // default MxRtyPSL
    pn532_packetbuffer[4] = mxRtyPassiveActivation;

    sendCommandCheckAck(pn532_packetbuffer, 5, 1000);
    // ignore response!
}


bool PN532::waitReady(uint16_t const timeout) const
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
bool PN532::sendCommandCheckAck(uint8_t const* cmd, uint8_t cmdlen, uint16_t timeout)
{
    // write the command
    writeCommand(cmd, cmdlen);

    return waitReady(timeout)
        && this->hasAck()
        && waitReady(timeout);
}

bool PN532::SAMConfig()
{
    if (! sendCommandCheckAck(CmdSamConfiguration(pn532_packetbuffer, SamMode_Normal, 20, SamIrq_Yes)))
        return false;

    // read data packet
    readData(pn532_packetbuffer, 8);

    // XXX: magic numbers
    return  (pn532_packetbuffer[5] == 0x15);
}

uint32_t PN532::authenticateBlock(
    uint8_t cardnumber /*1 or 2*/,
    uint32_t cid /*Card NUID*/,
    uint8_t blockaddress /*0 to 63*/,
    Key authtype,
    uint8_t const* keys)
{
    pn532_packetbuffer[0] = PN532_CMD_IN_DATA_EXCHANGE;
    pn532_packetbuffer[1] = cardnumber;  // either card 1 or 2 (tested for card 1)
    if(authtype == Key_A)
    {
        pn532_packetbuffer[2] = PN532_AUTH_WITH_KEYA;
    }
    else
    {
        pn532_packetbuffer[2] = PN532_AUTH_WITH_KEYB;
    }
    pn532_packetbuffer[3] = blockaddress; //This address can be 0-63 for MIFARE 1K card

    pn532_packetbuffer[4] = keys[0];
    pn532_packetbuffer[5] = keys[1];
    pn532_packetbuffer[6] = keys[2];
    pn532_packetbuffer[7] = keys[3];
    pn532_packetbuffer[8] = keys[4];
    pn532_packetbuffer[9] = keys[5];

    pn532_packetbuffer[10] = ((cid >> 24) & 0xFF);
    pn532_packetbuffer[11] = ((cid >> 16) & 0xFF);
    pn532_packetbuffer[12] = ((cid >> 8) & 0xFF);
    pn532_packetbuffer[13] = ((cid >> 0) & 0xFF);

    if (! sendCommandCheckAck(pn532_packetbuffer, 14, 1000))
        return false;

    // read data packet
    readData(pn532_packetbuffer, 2+6);

    for(int iter=0;iter<14;iter++)
    {
        Debug::print(pn532_packetbuffer[iter], HEX);
        Debug::print(" ");
    }
    Debug::println();
    // check some basic stuff

    Debug::println("AUTH");
    for(uint8_t i=0;i<2+6;i++)
    {
        Debug::print(pn532_packetbuffer[i], HEX); Debug::println(" ");
    }

    // XXX: magic numbers.
    return pn532_packetbuffer[6] == 0x41 && pn532_packetbuffer[7] == 0x00;
}

uint32_t PN532::readMemoryBlock(uint8_t cardnumber /*1 or 2*/,uint8_t blockaddress /*0 to 63*/, uint8_t* block) {
    pn532_packetbuffer[0] = PN532_CMD_IN_DATA_EXCHANGE;
    pn532_packetbuffer[1] = cardnumber;  // either card 1 or 2 (tested for card 1)
    pn532_packetbuffer[2] = PN532_MIFARE_READ;
    pn532_packetbuffer[3] = blockaddress; //This address can be 0-63 for MIFARE 1K card

    if (! sendCommandCheckAck(pn532_packetbuffer, 4, 1000))
        return false;

    // read data packet
    readData(pn532_packetbuffer, 18+6);
    // check some basic stuff
    Debug::println("READ");

    for(uint8_t i=8;i<18+6;i++)
    {
        block[i-8] = pn532_packetbuffer[i];
        Debug::print(pn532_packetbuffer[i], HEX); Debug::print(" ");
    }

    // XXX: Magic numbers
    return (pn532_packetbuffer[6] == 0x41) && (pn532_packetbuffer[7] == 0x00);
}

//Do not write to Sector Trailer Block unless you know what you are doing.
uint32_t PN532::writeMemoryBlock(uint8_t cardnumber /*1 or 2*/,uint8_t blockaddress /*0 to 63*/, uint8_t const* block) {
    pn532_packetbuffer[0] = PN532_CMD_IN_DATA_EXCHANGE;
    pn532_packetbuffer[1] = cardnumber;  // either card 1 or 2 (tested for card 1)
    pn532_packetbuffer[2] = PN532_MIFARE_WRITE;
    pn532_packetbuffer[3] = blockaddress;

    for(uint8_t byte=0; byte <16; byte++)
    {
        pn532_packetbuffer[4+byte] = block[byte];
    }

    if (! sendCommandCheckAck(pn532_packetbuffer, 20, 1000))
        return false;
    // read data packet
    readData(pn532_packetbuffer, 2+6);

    // check some basic stuff
    Debug::println("WRITE");
    for(uint8_t i=0;i<2+6;i++)
    {
        Debug::print(pn532_packetbuffer[i], HEX); Debug::println(" ");
    }

    // XXX: magic numbers
    return (pn532_packetbuffer[6] == 0x41) && (pn532_packetbuffer[7] == 0x00);
}

uint32_t PN532::readPassiveTargetID(BrTy cardbaudrate)
{
    uint32_t cid = 0;
    if (readPassiveTargetID(cardbaudrate, NULL, 0))
    {
        for (uint8_t i=0; i< pn532_packetbuffer[12]; i++)
        {
            cid <<= 8;
            cid |= pn532_packetbuffer[13+i];
            Debug::print(" 0x"); Debug::print(pn532_packetbuffer[13+i], HEX);
        }
    }
    return cid;
}

bool PN532::readPassiveTargetID(BrTy cardbaudrate, uint8_t* buffer, uint32_t bufferSize)
{
    if (! sendCommandCheckAck(CmdInListPassiveTarget(pn532_packetbuffer, 1, cardbaudrate, NULL)))
        return false;  // no cards read

    // read data packet
    readData(pn532_packetbuffer, 20);
    // check some basic stuff

    Debug::print("Found "); Debug::print(pn532_packetbuffer[7], DEC); Debug::println(" tags");

    if (pn532_packetbuffer[7] != 1)
    {
        return false;
    }

    uint16_t sens_res = pn532_packetbuffer[9];
    sens_res <<= 8;
    sens_res |= pn532_packetbuffer[10];
    Debug::print("Sens Response: 0x");  Debug::println(sens_res, HEX);
    Debug::print("Sel Response: 0x");  Debug::println(pn532_packetbuffer[11], HEX);

    for (uint8_t i = 0; i < pn532_packetbuffer[12] && i < bufferSize; ++i)
    {
        buffer[i] = pn532_packetbuffer[12+i];
    }

    Debug::println("TargetID");
    for (uint8_t i = 0; i < 20; ++i)
    {
        Debug::print(pn532_packetbuffer[i], HEX); Debug::println(" ");
    }

    return true;
}


/************** high level SPI */


bool PN532::hasAck() const
{
    uint8_t ackbuff[6];
    readData(ackbuff, 6);
    return (0 == memcmp(ackbuff, pn532ack, 6));
}

/************** mid level SPI */
//Send PN532_SPI_STATREAD byte to check the PN532 status 
//PN532 only answers PN532_SPI_READY once it has processed 
//the previous command either to send an ACK or to send the 
//final response to the command
uint8_t PN532::readStatus() const
{
    Bus::select();
    delay(2);
    uint8_t const x = Bus::transceiveByte(PN532_SPI_STATREAD);   
    Bus::deselect();
    return x;
}

//Start reading n bytes from PN532
//
//PN532 only answers PN532_SPI_READY once it has processed 
//the previous command either to send an ACK or to send the 
//final response to the command
void PN532::readData(uint8_t* buff, uint8_t n) const
{
    Bus::select();
    delay(2);
    Bus::dataReadFollows(); //read leading byte DR and discard

    Debug::print("Reading: ");

    for (uint8_t i = 0; i < n; ++i)
    {
        delay(1);
        buff[i] = Bus::readByte(); //read n bytes

        Debug::print(" 0x");
        Debug::print(buff[i], HEX);
    }

    Debug::println();

    Bus::deselect();
}

void PN532::writeCommand(uint8_t const* cmd, uint8_t cmdlen)
{
    ++cmdlen;

    Debug::print("\nSending: ");

    Bus::select();
    delay(2);     // or whatever the delay is for waking up the board
    Bus::dataWriteFollows();

    uint8_t checksum = PN532_PREAMBLE + PN532_STARTCODE1 + PN532_STARTCODE2;
    Bus::writeByte(PN532_PREAMBLE);
    Bus::writeByte(PN532_STARTCODE1);
    Bus::writeByte(PN532_STARTCODE2);

    Bus::writeByte(cmdlen);
    uint8_t const cmdlen_1 = ~cmdlen + 1;
    Bus::writeByte(cmdlen_1);

    Bus::writeByte(PN532_HOSTTOPN532);
    checksum += PN532_HOSTTOPN532;

    Debug::print(" 0x"); Debug::print(PN532_PREAMBLE, HEX);
    Debug::print(" 0x"); Debug::print(PN532_PREAMBLE, HEX);
    Debug::print(" 0x"); Debug::print(PN532_STARTCODE2, HEX);
    Debug::print(" 0x"); Debug::print(cmdlen, HEX);
    Debug::print(" 0x"); Debug::print(cmdlen_1, HEX);
    Debug::print(" 0x"); Debug::print(PN532_HOSTTOPN532, HEX);

    for (uint8_t i=0; i<cmdlen-1; i++)
    {
        Bus::writeByte(cmd[i]);
        checksum += cmd[i];

        Debug::print(" 0x"); Debug::print(cmd[i], HEX);
    }
    uint8_t const checksum_1 = ~checksum;
    Bus::writeByte(checksum_1);
    Bus::writeByte(PN532_POSTAMBLE);
    Bus::deselect();

    Debug::print(" 0x"); Debug::print(checksum_1, HEX);
    Debug::print(" 0x"); Debug::print(PN532_POSTAMBLE, HEX);
    Debug::println();
} 
