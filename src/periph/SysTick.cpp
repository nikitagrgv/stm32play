#include "SysTick.h"

#include "core/Globals.h"

#include <stm32f1xx.h>

namespace
{

uint32_t get_ctrl_flags(uint32_t setup_flags)
{
    uint32_t flags = 0;
    if (setup_flags & systick::ENABLE_INTERRUPT)
    {
        flags |= SysTick_CTRL_TICKINT_Msk;
    }
    return flags;
}

} // namespace

void systick::setupTimer(uint32_t frequency, uint32_t setup_flags)
{
    SysTick->LOAD = glob::SYS_FREQUENCY / frequency - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | get_ctrl_flags(setup_flags);
}

void systick::restartTimer()
{
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}
