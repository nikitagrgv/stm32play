#pragma once

#include "DeviceCMSIS.h"

namespace io
{

void setPrintUsart(USART_TypeDef *usart);

void printCharSync(char ch);

void printSync(const char *string);
void printSyncFmt(const char *fmt, ...);

} // namespace io
