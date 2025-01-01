#include "TIM.h"

#include "core/Globals.h"
#include "core/MicroAssert.h"

void tim::setupTimer(TIM_TypeDef *tim, uint32_t frequency, uint32_t reload_value, uint32_t flags)
{
    const uint32_t prescaler = (glob::SYS_FREQUENCY / frequency) - 1;

    MICRO_ASSERT(prescaler <= MAX_PRESCALER);
    MICRO_ASSERT(reload_value <= MAX_RELOAD_VALUE);

    const bool single_shot = flags & SINGLE_SHOT;

    tim->PSC = prescaler;
    tim->ARR = reload_value - single_shot;
    tim->CNT = 0;
    tim->EGR = TIM_EGR_UG; // Flush ARR and PSC!

    const uint32_t cr1 = flags;
    tim->CR1 = cr1;
}

void tim::runTimer(TIM_TypeDef *tim)
{
    tim->CNT = 0;
    tim->CR1 |= TIM_CR1_CEN;
}