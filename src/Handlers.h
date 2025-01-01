#pragma once

extern "C"
{
    void SysTick_Handler();
    void USART1_IRQHandler();
    void TIM2_IRQHandler();
    void EXTI0_IRQHandler();
}

namespace itr
{

enum class InterruptType
{
    SysTickHandler = 0,
    USART1Handler,
    TIM2Handler,
    EXTI0Handler,

    NUM_INTERRUPT_TYPES,
};

using HandlerFunc = void (*)(void *opaque);

void setHandler(InterruptType type, HandlerFunc func, void *opaque = nullptr);
void clearHandler(InterruptType type);

void setInterruptEnabled(InterruptType type, bool enabled);

} // namespace itr
