#include "HeapUsage.h"

extern "C"
{
    extern uint8_t _end;             // From the linker script
    extern uint8_t _estack;          // From the linker script
    extern uint32_t _Min_Stack_Size; // From the linker script
    extern uint8_t *__sbrk_heap_end; // From sysmem.c
}

void utils::getSbrkUsage(uint32_t &used, uint32_t &total)
{
    const uint32_t stack_limit = (uint32_t)&_estack - (uint32_t)&_Min_Stack_Size;
    const uint8_t *max_heap = (uint8_t *)stack_limit;

    total = max_heap - &_end;

    if (__sbrk_heap_end < &_end)
    {
        used = 0;
    }
    else
    {
        used = __sbrk_heap_end - &_end;
    }
}
