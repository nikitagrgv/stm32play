#include "Print.h"

#include <cstdarg>
#include <cstdio>
#include <stm32f103xb.h>

namespace
{

USART_TypeDef *PRINT_USART = nullptr;

}

void io::setPrintUsart(USART_TypeDef *usart)
{
    PRINT_USART = usart;
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
        // TODO: is it same to return here (va_list)?
        return;
    }

    va_list va;
    va_start(va, fmt);

    constexpr int BUFFER_SIZE = 1024;
    char buffer[BUFFER_SIZE];

    vsnprintf(buffer, BUFFER_SIZE, fmt, va);

    printSync(buffer);

    va_end(va);
}
