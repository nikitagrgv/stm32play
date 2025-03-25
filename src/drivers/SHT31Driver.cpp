#include "SHT31Driver.h"

#include "periph/I2C.h"
#include "utils/CRC.h"

SHT31Driver::SHT31Driver(I2C i2c)
    : i2c_(i2c)
{}

SHT31Driver::ErrorCode SHT31Driver::run(float &temperature, float &humidity)
{
    constexpr uint8_t sht31_address = 0x44;
    const uint8_t transmit_data[2] = {0x2C, 0x10};
    i2c::masterTransmitBlocking(i2c_, sht31_address, transmit_data, 2, TIMEOUT_MS);

    uint8_t receive_data[6] = {0, 0, 0, 0, 0, 0};
    i2c::masterReceiveBlocking(i2c_, sht31_address, receive_data, 6, TIMEOUT_MS);

    constexpr uint8_t crc8_poly = 0x31;
    constexpr uint8_t crc8_init = 0xFF;
    bool valid = true;
    valid &= utils::crc8(receive_data, 2, crc8_poly, crc8_init) == receive_data[2];
    valid &= utils::crc8(receive_data + 3, 2, crc8_poly, crc8_init) == receive_data[5];

    if (valid)
    {
        constexpr float divisor = 2 << 16 - 1;
        const uint16_t temp = (uint16_t)receive_data[0] << 8 | receive_data[1];
        const uint16_t hum = (uint16_t)receive_data[3] << 8 | receive_data[4];
        humidity = 100.0f * (float)hum / divisor;
        temperature = -45.0f + 175.0f * (float)temp / divisor;
    }

    return valid;
}