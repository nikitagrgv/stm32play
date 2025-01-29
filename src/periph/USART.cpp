#include "USART.h"

#include "DeviceCMSIS.h"
#include "core/Globals.h"

namespace
{

USART_TypeDef *get_usart_register(USART usart)
{
    switch (usart)
    {
    case USART::USART_1: return USART1; break;
    case USART::USART_2: return USART2; break;
#ifdef STM32F103
    case USART::USART_3: return USART3; break;
#endif
    default: MICRO_ASSERT(0); return nullptr;
    }
}

uint32_t get_cr1_flags(int setup_flags)
{
    uint32_t cr1 = 0;
    if (setup_flags & usart::ENABLE_TRANSMIT)
    {
        cr1 |= USART_CR1_TE;
    }
    if (setup_flags & usart::ENABLE_RECEIVE)
    {
        cr1 |= USART_CR1_RE;
    }
    if (setup_flags & usart::ENABLE_RECEIVE_INTERRUPT)
    {
        cr1 |= USART_CR1_RXNEIE;
    }
    return cr1;
}

} // namespace

void usart::setupUsart(USART usart, uint32_t baudrate, int setup_flags, bool enable)
{
    USART_TypeDef *usart_reg = get_usart_register(usart);

    uint32_t cr1 = enable ? USART_CR1_UE : 0;
    cr1 |= get_cr1_flags(setup_flags);

    const uint32_t brr = glob::SYS_FREQUENCY / baudrate;
    MICRO_ASSERT(brr < (1 << 16) && brr > 0);

    usart_reg->SR = 0; // clear flags
    usart_reg->BRR = brr;
    usart_reg->CR1 = cr1;
}

void usart::setUsartEnabled(USART usart, bool enabled)
{
    USART_TypeDef *usart_reg = get_usart_register(usart);

    const uint32_t old_cr1 = usart_reg->CR1;
    const uint32_t cr1 = enabled ? (old_cr1 | USART_CR1_UE) : (old_cr1 & ~(USART_CR1_UE));
    usart_reg->CR1 = cr1;
}
