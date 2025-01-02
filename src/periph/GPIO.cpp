#include "GPIO.h"

#include "core/Base.h"

#include <stm32f1xx.h>

namespace
{

FORCE_INLINE constexpr uint32_t get_mask(gpio::PinMode mode, int pos)
{
    MICRO_ASSERT(pos >= 0 && pos < 8);
    const int bit_pos = pos * 4;
    return (uint32_t)mode << bit_pos;
}

FORCE_INLINE constexpr uint32_t get_clear_mask(int pos)
{
    MICRO_ASSERT(pos >= 0 && pos < 8);
    return ~(0b1111UL << (pos * 4));
}

FORCE_INLINE constexpr GPIO_TypeDef *get_port_register(GPIOPort port)
{
    switch (port)
    {
    case GPIOPort::A: return GPIOA;
    case GPIOPort::B: return GPIOB;
    case GPIOPort::C: return GPIOC;
    case GPIOPort::D: return GPIOD;
    case GPIOPort::E: return GPIOE;
    default: MICRO_ASSERT(0); return nullptr;
    }
}

} // namespace

void gpio::setPinMode(GPIOPort port, int pin, PinMode mode)
{
    GPIO_TypeDef *port_reg = get_port_register(port);

    MICRO_ASSERT(pin >= 0 && pin < 16);
    const int is_high = pin >= 8;
    const int pos = pin % 8;
    auto &reg = is_high ? port_reg->CRH : port_reg->CRL;
    const uint32_t clear_mask = get_clear_mask(pos);
    const uint32_t mask = get_mask(mode, pos);
    reg = (reg & clear_mask) | mask;
}

void gpio::disablePin(GPIOPort port, int pin)
{
    setPinMode(port, pin, PinMode::InputFloating);
}

bool gpio::getPinInput(GPIOPort port, int pin)
{
    MICRO_ASSERT(pin >= 0 && pin < 16);
    const uint32_t mask = 1 << pin;
    return get_port_register(port)->IDR & mask;
}

void gpio::setPinOutput(GPIOPort port, int pin, bool value)
{
    MICRO_ASSERT(pin >= 0 && pin < 16);
    const uint32_t mask = value ? GPIO_BSRR_BS0 << pin : GPIO_BSRR_BR0 << pin;
    get_port_register(port)->BSRR = mask;
}

void gpio::setPinPullUpOrDown(GPIOPort port, int pin, PullUpOrDownMode mode)
{
    MICRO_ASSERT(pin >= 0 && pin < 16);
    setPinOutput(port, pin, mode == PullUpOrDownMode::Up);
}
