#pragma once

#include "core/MicroAssert.h"

#include <stm32f1xx.h>

namespace gpio
{

enum class PinMode
{
    InputFloating = 0b0100,
    InputPullUpOrDown = 0b1000,
    GeneralPushPull50MHz = 0b0011,
    GeneralOpenDrain50MHz = 0b0111,
    AlternatePushPull50MHz = 0b1011,
    AlternateOpenDrain50MHz = 0b1111,
};

void setPinMode(GPIO_TypeDef *port, int pin, PinMode mode);
void disablePin(GPIO_TypeDef *port, int pin);

inline void setPinOutput(GPIO_TypeDef *port, int pin, bool value)
{
    MICRO_ASSERT(pin < 16);
    const uint32_t mask = value ? GPIO_BSRR_BS0 << pin : GPIO_BSRR_BR0 << pin;
    port->BSRR = mask;
}

enum class PullUpOrDownMode
{
    Down = 0,
    Up = 1,
};
inline void setPinPullUpOrDown(GPIO_TypeDef *port, int pin, PullUpOrDownMode mode)
{
    MICRO_ASSERT(pin < 16);
    setPinOutput(port, pin, mode == PullUpOrDownMode::Up);
}

inline bool getPinInput(GPIO_TypeDef *port, int pin)
{
    MICRO_ASSERT(pin < 16);
    const uint32_t mask = 1 << pin;
    return port->IDR & mask;
}

} // namespace gpio
