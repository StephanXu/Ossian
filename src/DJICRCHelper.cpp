#include "DJICRCHelper.hpp"

auto DJICRCHelper::GetCRC8Checksum(const unsigned char* pchMessage, size_t length,
	unsigned char ucCRC8) noexcept -> unsigned char
{
	unsigned char tableIndex;
	while (length--)
	{
		tableIndex = ucCRC8 ^ (*pchMessage++);
		ucCRC8 = m_CRC8Table[tableIndex];
	}
	return ucCRC8;
}

/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length=Data+checksum
** Output: True or False(CRC Verify Result)
*/
auto DJICRCHelper::VerifyCRC8Checksum(const unsigned char* pchMessage, size_t length) noexcept -> bool
{
	if (pchMessage == nullptr || length <= 2)
		return false;
	const auto ucExpected = GetCRC8Checksum(pchMessage, length - 1, m_CRC8Init);
	return (ucExpected == pchMessage[length - 1]);
}

/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length=Data+checksum
** Output: True or False(CRC Verify Result)
*/
auto DJICRCHelper::AppendCRC8Checksum(unsigned char* pchMessage, size_t length) noexcept -> void
{
	if (pchMessage == nullptr || length <= 2)
		return;
	const auto ucCRC = GetCRC8Checksum(static_cast<unsigned char*>(pchMessage), length - 1, m_CRC8Init);
	pchMessage[length - 1] = ucCRC;
}

/*
** Descriptions: CRC16 checksum function
** Input: Data to check, Stream length, initialized checksum
** Output: CRC checksum
*/
auto DJICRCHelper::GetCRC16Checksum(const uint8_t* pchMessage, size_t length, uint16_t wCRC) noexcept -> unsigned short
{
	if (pchMessage == nullptr)
	{
		return 0xFFFF;
	}
	while (length--)
	{
		const auto chData = *pchMessage++;
		wCRC = (static_cast<uint16_t>(wCRC) >> 8) ^ m_WideCRCTable[(static_cast<uint16_t>(wCRC) ^ static_cast<uint16_t>(chData)) & 0x00ff];
	}
	return wCRC;
}

/*
** Descriptions:CRC16 Verify function
** Input:Data to Verify, Stream length= Data + checksum
** Output:True or False (CRC Verify Result)
*/
auto DJICRCHelper::VerifyCRC16Checksum(const uint8_t* pchMessage, size_t length) noexcept -> bool
{
	if (pchMessage == nullptr || length <= 2)
		return false;
	const auto wExpected = GetCRC16Checksum(pchMessage, length - 2, m_CRC16Init);
	return (wExpected & 0xff) == pchMessage[length - 2] && (wExpected >> 8 & 0xff) == pchMessage[length - 1];
}

/*
** Descriptions: append CRC16 to the end of data
** Input:Data to CRC and append, Stream length = Data +checksum
** Output: True or False (CRC Verify Result)
*/
auto DJICRCHelper::AppendCRC16Checksum(uint8_t* pchMessage, size_t length) noexcept -> void
{
	if (pchMessage == nullptr || length <= 2)
	{
		return;
	}
	const auto wCRC = GetCRC16Checksum(static_cast<uint8_t*>(pchMessage), length - 2, m_CRC16Init);
	pchMessage[length - 2] = static_cast<uint8_t>(wCRC & 0x00ff);
	pchMessage[length - 1] = static_cast<uint8_t>((wCRC >> 8) & 0x00ff);
}
