#include "IRQ.h"

#include "DeviceCMSIS.h"
#include "core/Base.h"
#include "core/Globals.h"
#include "core/MicroAssert.h"

namespace
{

struct FuncWithOpaque
{
    irq::HandlerFunc func{};
    void *opaque{};
};

FuncWithOpaque handlers[(int)InterruptType::NUM_INTERRUPT_TYPES];

FORCE_INLINE void call_handler(InterruptType type)
{
    const FuncWithOpaque &fwo = handlers[(int)type];
    if (!fwo.func)
    {
        return;
    }
    (*fwo.func)(fwo.opaque);
}

FORCE_INLINE IRQn_Type get_irqn_by_type(InterruptType type)
{
#ifdef STM32F103
    switch (type)
    {
    case InterruptType::SysTickIRQ: return SysTick_IRQn;

    case InterruptType::USART1IRQ: return USART1_IRQn;

    case InterruptType::TIM1_UP_IRQ: return TIM1_UP_IRQn;
    case InterruptType::TIM2IRQ: return TIM2_IRQn;
    case InterruptType::TIM3IRQ: return TIM3_IRQn;
    case InterruptType::TIM4IRQ: return TIM4_IRQn;

    case InterruptType::EXTI0IRQ: return EXTI0_IRQn;
    case InterruptType::EXTI1IRQ: return EXTI1_IRQn;
    case InterruptType::EXTI2IRQ: return EXTI2_IRQn;
    case InterruptType::EXTI3IRQ: return EXTI3_IRQn;
    case InterruptType::EXTI4IRQ: return EXTI4_IRQn;
    case InterruptType::EXTI5_9_IRQn: return EXTI9_5_IRQn;
    case InterruptType::EXTI10_15_IRQn: return EXTI15_10_IRQn;

    default: MICRO_ASSERT(0); return SysTick_IRQn;
    }
#elifdef STM32F401
    switch (type)
    {
    case InterruptType::SysTickIRQ: return SysTick_IRQn;

    case InterruptType::USART1IRQ: return USART1_IRQn;

    case InterruptType::TIM1_UP_IRQ: return TIM1_UP_TIM10_IRQn;

    case InterruptType::TIM2IRQ: return TIM2_IRQn;
    case InterruptType::TIM3IRQ: return TIM3_IRQn;
    case InterruptType::TIM4IRQ: return TIM4_IRQn;

    case InterruptType::EXTI0IRQ: return EXTI0_IRQn;
    case InterruptType::EXTI1IRQ: return EXTI1_IRQn;
    case InterruptType::EXTI2IRQ: return EXTI2_IRQn;
    case InterruptType::EXTI3IRQ: return EXTI3_IRQn;
    case InterruptType::EXTI4IRQ: return EXTI4_IRQn;
    case InterruptType::EXTI5_9_IRQn: return EXTI9_5_IRQn;
    case InterruptType::EXTI10_15_IRQn: return EXTI15_10_IRQn;

    default: MICRO_ASSERT(0); return SysTick_IRQn;
    }
#else
    #error
#endif
}

} // namespace

extern "C"
{

    // TODO! REVERT
    void USART1_IRQHandler()
    {
        call_handler(InterruptType::USART1IRQ);
    }

    void TIM2_IRQHandler()
    {
        call_handler(InterruptType::TIM2IRQ);
    }

    void EXTI0_IRQHandler()
    {
        call_handler(InterruptType::EXTI0IRQ);
    }

    void EXTI1_IRQHandler()
    {
        call_handler(InterruptType::EXTI1IRQ);
    }

    void EXTI2_IRQHandler()
    {
        call_handler(InterruptType::EXTI2IRQ);
    }

    void EXTI3_IRQHandler()
    {
        call_handler(InterruptType::EXTI3IRQ);
    }

    void EXTI4_IRQHandler()
    {
        call_handler(InterruptType::EXTI4IRQ);
    }

    void EXTI9_5_IRQHandler()
    {
        call_handler(InterruptType::EXTI5_9_IRQn);
    }

    void EXTI15_10_IRQHandler()
    {
        call_handler(InterruptType::EXTI10_15_IRQn);
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
