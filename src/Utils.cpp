#include "Utils.h"

#include "Globals.h"

void str_utils::sleepMsec(uint32_t msec)
{
    const uint32_t end_msec = glob::total_msec + msec;
    while (glob::total_msec < end_msec)
    {}
}