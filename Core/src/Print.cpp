#include "Print.h"

#include <cstdarg>
#include <cstdio>
#include <stm32f103xb.h>

namespace
{

USART_TypeDef *PRINT_USART = nullptr;

constexpr int PRINT_BUFFER_SIZE = 1024;

} // namespace

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
        // TODO: is it same to return here (va_list)?
        return;
    }

    va_list va;
    va_start(va, fmt);

    char buffer[PRINT_BUFFER_SIZE];

    vsnprintf(buffer, PRINT_BUFFER_SIZE, fmt, va);

    printSync(buffer);

    va_end(va);
}

void io::printLineSync(const char *string)
{
    printSync(string);
    printCharSync('\n');
}

void io::printLineSyncFmt(const char *fmt, ...)
{
    if (!PRINT_USART)
    {
        // TODO: is it same to return here (va_list)?
        return;
    }

    va_list va;
    va_start(va, fmt);

    char buffer[PRINT_BUFFER_SIZE];

    vsnprintf(buffer, PRINT_BUFFER_SIZE, fmt, va);

    printLineSync(buffer);

    va_end(va);
}
