#pragma once

#include "PeriphBase.h"

#include <cstdint>

namespace usart
{

enum SetupFlags : uint32_t
{
    ENABLE_TRANSMIT = 1 << 0,
    ENABLE_RECEIVE = 1 << 1,
    ENABLE_RECEIVE_INTERRUPT = 1 << 2,
};

void setupUsart(USART usart, uint32_t baudrate, int setup_flags, bool enable = true);

void setUsartEnabled(USART usart, bool enabled);

} // namespace usart
