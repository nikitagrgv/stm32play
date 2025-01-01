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

enum class HandlerType
{
    SysTickHandler = 0,
    USART1Handler,
    TIM2Handler,
    EXTI0Handler,

    NUM_HANDLERS,
};

using HandlerFunc = void (*)(void *opaque);

void setHandler(HandlerType type, HandlerFunc func, void *opaque = nullptr);
void clearHandler(HandlerType type);

} // namespace itr
