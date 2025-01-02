#include "TIM.h"

#include "core/Base.h"
#include "core/Globals.h"
#include "core/MicroAssert.h"

namespace
{

FORCE_INLINE uint32_t get_cr1_flags(uint32_t setup_flags)
{
    uint32_t flags = 0;
    if (setup_flags & tim::SINGLE_SHOT)
    {
        flags |= TIM_CR1_OPM;
    }
    return flags;
}

FORCE_INLINE uint32_t get_dier_flags(uint32_t setup_flags)
{
    uint32_t flags = 0;
    if (setup_flags & tim::ENABLE_UPDATE_INTERRUPT)
    {
        flags |= TIM_DIER_UIE;
    }
    return flags;
}

} // namespace

void tim::setupTimer(TIM_TypeDef *tim, uint32_t frequency, uint32_t reload_value, uint32_t setup_flags)
{
    const uint32_t prescaler = (glob::SYS_FREQUENCY / frequency) - 1;

    MICRO_ASSERT(prescaler <= MAX_PRESCALER);
    MICRO_ASSERT(reload_value <= MAX_RELOAD_VALUE);

    const bool single_shot = setup_flags & SINGLE_SHOT;

    tim->PSC = prescaler;
    tim->ARR = reload_value - single_shot;
    tim->CNT = 0;
    tim->EGR = TIM_EGR_UG; // Trigger interrupt to Flush ARR and PSC!
    tim->DIER = get_dier_flags(setup_flags);
    tim->CR1 = get_cr1_flags(setup_flags);

    tim->SR = 0; // Clear interrupt flags
}

void tim::restartTimer(TIM_TypeDef *tim)
{
    tim->SR = 0; // Clear interrupt flags
    tim->CNT = 0;
    tim->CR1 |= TIM_CR1_CEN;
}

void tim::stopTimer(TIM_TypeDef *tim)
{
    tim->CR1 &= ~TIM_CR1_CEN;
    tim->SR = 0; // Clear interrupt flags
}

uint32_t tim::getTimerValue(TIM_TypeDef *tim)
{
    return tim->CNT;
}

bool tim::checkPendingUpdateAndClear(TIM_TypeDef *tim)
{
    if (tim->SR & TIM_SR_UIF)
    {
        tim->SR = ~TIM_SR_UIF; // NOTE: write 0 to clear
        return true;
    }
    return false;
}

InterruptType tim::getUpdateInterruptType(const TIM_TypeDef *tim)
{
    if (tim == TIM1)
    {
        return InterruptType::TIM1_UP_IRQ;
    }
    if (tim == TIM2)
    {
        return InterruptType::TIM2IRQ;
    }
    if (tim == TIM3)
    {
        return InterruptType::TIM3IRQ;
    }
    if (tim == TIM4)
    {
        return InterruptType::TIM4IRQ;
    }

    MICRO_ASSERT(0);
    return InterruptType::SysTickIRQ;
}
