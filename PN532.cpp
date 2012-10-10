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

using namespace pn532;

uint8_t PN532Base_::packetBuffer[PacketBufferSize];

GetFirmwareVersion::GetFirmwareVersion(uint8_t* buf)
    : ptr(buf)
{
    buf[0] = Code;
}

RfConfiguration::RfConfiguration(uint8_t* buf, RfConfigurationItem cfgItem, void const* cfgData)
	: ptr(buf)
{
	*(buf++) = Code;
	*(buf++) = cfgItem;
	uint8_t size;
	switch (cfgItem)
	{
	default: return; // TODO: halt();
	case RfConfigurationItem_RfField: // nobreak
	case RfConfigurationItem_MaxRtyCom:
		size = 1;
		break;

	case RfConfigurationItem_VariousTimings: // nobreak
	case RfConfigurationItem_MaxRetries: // nobreak
	case RfConfigurationItem_AnalogSettings_for_TypeB:
		size = 3;
		break;

	case RfConfigurationItem_AnalogSettings_for_106kbpsTypeA:
		size = 11; break;

	case RfConfigurationItem_AnalogSettings_for_212_424kbps:
		size = 8; break;

	case RfConfigurationItem_AnalogSettings_for_212_424_848:
		size = 9; break;
	}
	memcpy(buf, cfgData, size);

	length_ = size + 2;
}

SamConfiguration::SamConfiguration(uint8_t* buf, SamMode samMode, uint8_t timeout_50ms, SamIrq samIrq)
    : ptr(buf)
{
    *(buf++) = Code;
    *(buf++) = samMode;
    *(buf++) = timeout_50ms;
    *(buf++) = samIrq;
}

InListPassiveTarget::InListPassiveTarget(uint8_t* buf, uint8_t maxTags, BrTy brTy, uint8_t const* /*initiatorData*/)
    : ptr(buf)
{
    *(buf++) = Code;
    *(buf++) = maxTags;
    *(buf++) = brTy;

    // TODO:initiator data.
}

