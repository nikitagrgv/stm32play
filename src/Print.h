
#pragma once
#include <stm32f103xb.h>

namespace io
{

void setPrintUsart(USART_TypeDef *usart);

void printCharSync(char ch);

void printSync(const char *string);
void printSyncFmt(const char *fmt, ...);

} // namespace io
