#include "DeviceCMSIS.h"
#include "core/Base.h"
#include "periph/GPIO.h"

namespace
{

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

FORCE_INLINE constexpr uint32_t get_mask(uint32_t mode, int pos)
{
    MICRO_ASSERT(pos >= 0 && pos < 8);
    const int bit_pos = pos * 4;
    return mode << bit_pos;
}

FORCE_INLINE constexpr uint32_t get_clear_mask(int pos)
{
    MICRO_ASSERT(pos >= 0 && pos < 8);
    return ~(0b1111UL << (pos * 4));
}

FORCE_INLINE constexpr void configure(Pin pin, uint32_t mode)
{
    GPIO_TypeDef *port_reg = get_port_register(pin.port);

    MICRO_ASSERT(pin.isValid());
    const int is_high = pin.num >= 8;
    const int pos = pin.num % 8;
    auto &reg = is_high ? port_reg->CRH : port_reg->CRL;
    const uint32_t clear_mask = get_clear_mask(pos);
    const uint32_t mask = get_mask(mode, pos);
    reg = (reg & clear_mask) | mask;
}

} // namespace

void gpio::configureOutput(Pin pin, OutputSpeed speed, PullMode pull_mode) {}

void gpio::configureInput(Pin pin, PullMode pull_mode) {}

void gpio::disablePin(Pin pin)
{
    configureInput(pin, PullMode::None);
}

bool gpio::getPinInput(Pin pin)
{
    MICRO_ASSERT(pin.isValid());
    const uint32_t mask = 1 << pin.num;
    return get_port_register(pin.port)->IDR & mask;
}

void gpio::setPinOutput(Pin pin, bool value)
{
    MICRO_ASSERT(pin.isValid());
    const uint32_t mask = value ? GPIO_BSRR_BS0 << pin.num : GPIO_BSRR_BR0 << pin.num;
    get_port_register(pin.port)->BSRR = mask;
}
