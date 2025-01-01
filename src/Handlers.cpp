#include "Handlers.h"

#include "core/Base.h"
#include "core/Globals.h"
#include "core/MicroAssert.h"

#include <stm32f1xx.h>

namespace
{

struct FuncWithOpaque
{
    itr::HandlerFunc func{};
    void *opaque{};
};

FuncWithOpaque handlers[(int)itr::InterruptType::NUM_INTERRUPT_TYPES];

FORCE_INLINE void call_handler(itr::InterruptType type)
{
    const FuncWithOpaque &fwo = handlers[(int)type];
    if (!fwo.func)
    {
        return;
    }
    (*fwo.func)(fwo.opaque);
}

FORCE_INLINE IRQn_Type get_irqn_by_type(itr::InterruptType type)
{
    switch (type)
    {
    case itr::InterruptType::SysTickHandler: return TIM2_IRQn;
    case itr::InterruptType::USART1Handler: return USART1_IRQn;
    case itr::InterruptType::TIM2Handler: return TIM2_IRQn;
    case itr::InterruptType::EXTI0Handler: return EXTI0_IRQn;
    default: MICRO_ASSERT(0); return HardFault_IRQn;
    }
}

} // namespace

extern "C"
{
    void SysTick_Handler()
    {
        call_handler(itr::InterruptType::SysTickHandler);
    }

    void USART1_IRQHandler()
    {
        call_handler(itr::InterruptType::USART1Handler);
    }

    void TIM2_IRQHandler()
    {
        call_handler(itr::InterruptType::TIM2Handler);
    }

    void EXTI0_IRQHandler()
    {
        call_handler(itr::InterruptType::EXTI0Handler);
    }
}

void itr::setHandler(InterruptType type, HandlerFunc func, void *opaque)
{
    FuncWithOpaque &fwo = handlers[(int)type];
    fwo.func = func;
    fwo.opaque = opaque;
}

void itr::clearHandler(InterruptType type)
{
    setHandler(type, nullptr, nullptr);
}

void itr::setInterruptEnabled(InterruptType type, bool enabled)
{
    const IRQn_Type irqn = get_irqn_by_type(type);
}

void itr::setInterruptsEnabled(bool enabled)
{
    if (enabled)
    {
        __enable_irq();
    }
    else
    {
        __disable_irq();
    }
}

void itr::disableInterrupts()
{
    __disable_irq();
}

void itr::enableInterrupts()
{
    __enable_irq();
}
