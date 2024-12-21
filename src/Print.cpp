#include "Print.h"

#include "MicroAssert.h"

#include <cstdarg>
#include <printf.h>
#include <stm32f103xb.h>

namespace
{

USART_TypeDef *PRINT_USART = nullptr;

} // namespace

// implement symbol from mpaland/printf library
void _putchar(char character)
{
    MICRO_ASSERT(PRINT_USART);
    while (!(USART1->SR & USART_SR_TXE))
    {}
    USART1->DR = character;
}

void io::setPrintUsart(USART_TypeDef *usart)
{
    PRINT_USART = usart;
}

void io::printCharSync(char ch)
{
    if (!PRINT_USART)
    {
        return;
    }


    while (!(USART1->SR & USART_SR_TXE))
    {}
    USART1->DR = ch;
}

void io::printSync(const char *string)
{
    if (!PRINT_USART)
    {
        return;
    }

    const char *p = string;
    while (*p)
    {
        while (!(USART1->SR & USART_SR_TXE))
        {}
        USART1->DR = *p++;
    }
}

void io::printSyncFmt(const char *fmt, ...)
{
    if (!PRINT_USART)
    {
        // TODO: is it safe to return here (va_list)?
        return;
    }

    va_list va;
    va_start(va, fmt);
    vprintf(fmt, va);
    va_end(va);
}
