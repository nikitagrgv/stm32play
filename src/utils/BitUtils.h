#pragma once

#include "core/Base.h"

#include <cstdint>

namespace utils
{


FORCE_INLINE uint8_t flipByte(uint8_t byte)
{
    uint8_t result = 0;
    for (int i = 0; i < 8; ++i)
    {
        result |= ((byte >> i) & 1) << (7 - i);
    }
    return result;
}

} // namespace utils
