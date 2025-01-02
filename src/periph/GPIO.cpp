#include "GPIO.h"

#include "core/Base.h"

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

} // namespace

void gpio::setPinMode(GPIO_TypeDef *port, int pin, PinMode mode)
{
    MICRO_ASSERT(pin < 16);
    const int is_high = pin >= 8;
    const int pos = pin % 8;
    auto &reg = is_high ? port->CRH : port->CRL;
    const uint32_t clear_mask = get_clear_mask(pos);
    const uint32_t mask = get_mask(mode, pos);
    reg = (reg & clear_mask) | mask;
}

void gpio::disablePin(GPIO_TypeDef *port, int pin)
{
    setPinMode(port, pin, PinMode::InputFloating);
}
