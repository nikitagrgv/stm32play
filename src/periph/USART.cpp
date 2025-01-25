#include "USART.h"

void usart::setupUsart(USART_TypeDef *usart, uint32_t baudrate, int setup_flags, bool enable)
{
    uint32_t cr1 = enable ? USART_CR1_UE : 0;
    cr1 |= (uint32_t)setup_flags;

    const uint32_t brr = glob::SYS_FREQUENCY / baudrate;
    MICRO_ASSERT(brr < (1 << 16) && brr > 0);

    usart->SR = 0; // clear flags
    usart->BRR = brr;
    usart->CR1 = cr1;
}

void usart::setUsartEnabled(USART_TypeDef *usart, bool enabled)
{
    const uint32_t old_cr1 = usart->CR1;
    const uint32_t cr1 = enabled ? (old_cr1 | USART_CR1_UE) : (old_cr1 & ~(USART_CR1_UE));
    usart->CR1 = cr1;
}
