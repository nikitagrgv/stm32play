#include "Print.h"

#include "MicroAssert.h"

#include <cstdarg>
#include <cstdio>
#include <stm32f103xb.h>

namespace
{

USART_TypeDef *PRINT_USART = nullptr;

} // namespace

extern "C"
{
    // implement symbol from syscall.c
    // TODO: implement __io_getchar too
    int __io_putchar(int ch)
    {
        MICRO_ASSERT(PRINT_USART);
        while (!(USART1->SR & USART_SR_TXE))
        {}
        USART1->DR = ch;
        return ch;
    }
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
