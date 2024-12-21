#include "Handlers.h"

#include "Globals.h"

extern "C"
{
    void SysTick_Handler()
    {
        ++glob::total_msec;
    }
}
