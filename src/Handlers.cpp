#include "Handlers.h"

#include "core/Globals.h"

extern "C"
{
    void SysTick_Handler()
    {
        ++glob::total_msec;
    }
}
