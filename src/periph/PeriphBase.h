#pragma once

enum class GPIOPort
{
    A = 0,
    B,
    C,
    D,
    E,
};

enum class InterruptType
{
    SysTickIRQ = 0,
    USART1IRQ,
    TIM2IRQ,

    EXTI0IRQ,
    EXTI1IRQ,
    EXTI2IRQ,
    EXTI3IRQ,
    EXTI4IRQ,
    EXTI5_9_IRQn,
    EXTI10_15_IRQn,

    NUM_INTERRUPT_TYPES,
};
