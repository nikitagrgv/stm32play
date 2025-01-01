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

constexpr uint32_t getGPIOMask(PinMode mode, int pos)
{
    MICRO_ASSERT(pos < 8);
    const int bit_pos = pos * 4;
    return (uint32_t)mode << bit_pos;
}

constexpr uint32_t getGPIOClearMask(int pos)
{
    MICRO_ASSERT(pos < 8);
    return ~(0b1111UL << (pos * 4));
}

inline void setPinMode(GPIO_TypeDef *port, int pin, PinMode mode)
{
    const int is_high = pin >= 8;
    const int pos = pin % 8;
    auto &reg = is_high ? port->CRH : port->CRL;
    const uint32_t clear_mask = getGPIOClearMask(pos);
    const uint32_t mask = getGPIOMask(mode, pos);
    reg = (reg & clear_mask) | mask;
}

inline void setPinOutput(GPIO_TypeDef *port, int pin, bool value)
{
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
    setPinOutput(port, pin, mode == PullUpOrDownMode::Up);
}

} // namespace gpio
