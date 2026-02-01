#include "crc_x25.hpp"

// CRC-16/X.25 implementation
uint16_t crcX25(const uint8_t *data, uint16_t length) {
	uint16_t crc = 0xFFFFU;

	for (uint16_t i = 0U; i < length; ++i)
	{
		crc ^= data[i];
		for (uint8_t bit = 0U; bit < 8U; ++bit)
		{
			if (crc & 1U)
				crc = (crc >> 1U) ^ 0x8408U;
			else
				crc >>= 1U;
		}
	}

	return crc;
}
