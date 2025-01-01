#pragma once

extern "C"
{
    void SysTick_Handler();
    void USART1_IRQHandler();
    void TIM2_IRQHandler();
    void EXTI0_IRQHandler();
}

namespace irq
{

enum class InterruptType
{
    SysTickIRQ = 0,
    USART1IRQ,
    TIM2IRQ,
    EXTI0IRQ,

    NUM_INTERRUPT_TYPES,
};

using HandlerFunc = void (*)(void *opaque);

void setHandler(InterruptType type, HandlerFunc func, void *opaque = nullptr);
void clearHandler(InterruptType type);

void setInterruptEnabled(InterruptType type, bool enabled);
void disableInterrupt(InterruptType type);
void enableInterrupt(InterruptType type);

void setInterruptsEnabled(bool enabled);
void disableInterrupts();
void enableInterrupts();

} // namespace irq
