#pragma once

#include "core/Globals.h"
#include "core/MicroAssert.h"

#include <cstdint>
#include <stm32f1xx.h>

namespace usart
{

enum SetupFlags : uint32_t
{
    ENABLE_TRANSMIT = USART_CR1_TE,
    ENABLE_RECEIVE = USART_CR1_RE,
    ENABLE_RECEIVE_INTERRUPT = USART_CR1_RXNEIE,
};

void setupUsart(USART_TypeDef *usart, uint32_t baudrate, int setup_flags, bool enable = true);

void setUsartEnabled(USART_TypeDef *usart, bool enabled);

} // namespace usart
