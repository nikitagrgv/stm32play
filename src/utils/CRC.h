#pragma once

#include <cstdint>

namespace utils
{

uint8_t crc8(const uint8_t *data, int len, uint8_t polynomial, uint8_t init);

} // namespace utils