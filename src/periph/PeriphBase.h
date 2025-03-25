#pragma once

#include "core/Base.h"
#include "core/MicroAssert.h"

enum class GPIOPort : uint8_t
{
    A = 0,
    B = 1,
    C = 2,
    D = 3,
    E = 4,
};

enum class USART : uint8_t
{
    USART_1,
    USART_2,

#ifdef STM32F103
    USART_3,
#endif
};

enum class I2C : uint8_t
{
    I2C_1,
    I2C_2,
    I2C_3,
};

enum class InterruptType : uint8_t
{
    SysTickIRQ = 0,

    USART1IRQ,

    TIM1_UP_IRQ,
    TIM2IRQ,
    TIM3IRQ,
    TIM4IRQ,

    EXTI0IRQ,
    EXTI1IRQ,
    EXTI2IRQ,
    EXTI3IRQ,
    EXTI4IRQ,
    EXTI5_9_IRQn,
    EXTI10_15_IRQn,

    NUM_INTERRUPT_TYPES,
};

struct Pin
{
    FORCE_INLINE constexpr Pin() = default;

    FORCE_INLINE constexpr Pin(GPIOPort port, int num)
        : port(port)
        , num(num)
    {
        MICRO_ASSERT(num < 16);
    }

    FORCE_INLINE constexpr bool isValid() const { return num < 16; }

    GPIOPort port{GPIOPort::A};
    uint8_t num{0};
};
