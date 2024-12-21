#include "MicroAssert.h"

 void micro_assert_failed(const char *file, int line, const char *func, const char *expr)
{
    __disable_irq(); // disable interrupts
    while (true)
    {}
}
