#include "Sleep.h"

#include "Globals.h"

void utils::sleepMsec(uint32_t msec)
{
    const uint32_t end_msec = glob::total_msec + msec;
    while (glob::total_msec < end_msec)
    {}
}