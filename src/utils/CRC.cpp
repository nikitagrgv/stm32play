#include "CRC.h"

uint8_t utils::crc8(const uint8_t *data, int len, uint8_t polynomial, uint8_t init)
{
    uint8_t crc = init;

    for (int b = 0; b < len; ++b)
    {
        const uint8_t byte = data[b];

        crc ^= byte;
        for (int i = 0; i < 8; ++i)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ polynomial;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}