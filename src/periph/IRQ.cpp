#include "IRQ.h"

#include "core/Base.h"
#include "core/Globals.h"
#include "core/MicroAssert.h"

#include <stm32f1xx.h>

namespace
{

struct FuncWithOpaque
{
    irq::HandlerFunc func{};
    void *opaque{};
};

FuncWithOpaque handlers[(int)irq::InterruptType::NUM_INTERRUPT_TYPES];

FORCE_INLINE void call_handler(irq::InterruptType type)
{
    const FuncWithOpaque &fwo = handlers[(int)type];
    if (!fwo.func)
    {
        return;
    }
    (*fwo.func)(fwo.opaque);
}

FORCE_INLINE IRQn_Type get_irqn_by_type(irq::InterruptType type)
{
    switch (type)
    {
    case irq::InterruptType::SysTickIRQ: return SysTick_IRQn;
    case irq::InterruptType::USART1IRQ: return USART1_IRQn;
    case irq::InterruptType::TIM2IRQ: return TIM2_IRQn;
    case irq::InterruptType::EXTI0IRQ: return EXTI0_IRQn;
    case irq::InterruptType::EXTI1IRQ: return EXTI1_IRQn;
    case irq::InterruptType::EXTI2IRQ: return EXTI2_IRQn;
    case irq::InterruptType::EXTI3IRQ: return EXTI3_IRQn;
    default: MICRO_ASSERT(0); return HardFault_IRQn;
    }
}

} // namespace

extern "C"
{
    void SysTick_Handler()
    {
        call_handler(irq::InterruptType::SysTickIRQ);
    }

    void USART1_IRQHandler()
    {
        call_handler(irq::InterruptType::USART1IRQ);
    }

    void TIM2_IRQHandler()
    {
        call_handler(irq::InterruptType::TIM2IRQ);
    }

    void EXTI0_IRQHandler()
    {
        call_handler(irq::InterruptType::EXTI0IRQ);
    }

    void EXTI1_IRQHandler()
    {
        call_handler(irq::InterruptType::EXTI1IRQ);
    }

    void EXTI2_IRQHandler()
    {
        call_handler(irq::InterruptType::EXTI2IRQ);
    }

    void EXTI3_IRQHandler()
    {
        call_handler(irq::InterruptType::EXTI3IRQ);
    }
}

void irq::setHandler(InterruptType type, HandlerFunc func, void *opaque)
{
    FuncWithOpaque &fwo = handlers[(int)type];
    fwo.func = func;
    fwo.opaque = opaque;
}

void irq::clearHandler(InterruptType type)
{
    setHandler(type, nullptr, nullptr);
}

void irq::setInterruptEnabled(InterruptType type, bool enabled)
{
    const IRQn_Type irqn = get_irqn_by_type(type);
    if (enabled)
    {
        NVIC_EnableIRQ(irqn);
    }
    else
    {
        NVIC_DisableIRQ(irqn);
    }
}

void irq::disableInterrupt(InterruptType type)
{
    const IRQn_Type irqn = get_irqn_by_type(type);
    NVIC_DisableIRQ(irqn);
}

void irq::enableInterrupt(InterruptType type)
{
    const IRQn_Type irqn = get_irqn_by_type(type);
    NVIC_EnableIRQ(irqn);
}

void irq::setInterruptsEnabled(bool enabled)
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

void irq::disableInterrupts()
{
    __disable_irq();
}

void irq::enableInterrupts()
{
    __enable_irq();
}
