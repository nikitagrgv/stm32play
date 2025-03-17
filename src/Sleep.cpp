#include "Sleep.h"

#include "core/Globals.h"
#include "periph/TIM.h"

void utils::sleepMsec(uint32_t msec)
{
    const uint32_t end_msec = glob::total_msec + msec + 1;
    while (glob::total_msec < end_msec)
    {}
}

void utils::sleepMsec(TIM_TypeDef *timer, uint32_t msec)
{
    constexpr uint32_t frequency = 2'000; // can't set 1'000, too small
    const uint32_t reload_value = msec * 2;
    tim::setupTimer(timer, frequency, reload_value, tim::SINGLE_SHOT);
    tim::restartTimer(timer);
    while (!tim::checkPendingUpdateAndClear(timer))
    {}
    tim::stopTimer(timer);
}

void utils::sleepUsec(TIM_TypeDef *timer, uint32_t usec)
{
    constexpr uint32_t frequency = 1'000'000;
    const uint32_t reload_value = usec;
    tim::setupTimer(timer, frequency, reload_value, tim::SINGLE_SHOT);
    tim::restartTimer(timer);
    while (!tim::checkPendingUpdateAndClear(timer))
    {}
    tim::stopTimer(timer);
}
