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

FORCE_INLINE constexpr uint32_t number_to_mask(int k)
{
    MICRO_ASSERT(k < 32);
    return (1U << k) - 1;
}

FORCE_INLINE constexpr void get_masks(uint32_t base_mask, int pos, int size, uint32_t &mask, uint32_t &clear_mask)
{
    MICRO_ASSERT(pos >= 0 && pos < 16);
    MICRO_ASSERT(pos * size < 32);
    const int bit_pos = pos * size;
    mask = base_mask << bit_pos;
    clear_mask = ~(number_to_mask(size) << bit_pos);
    MICRO_ASSERT((mask & clear_mask) == 0);
}

void configure(Pin pin, uint32_t mode, uint32_t otype, uint32_t ospeed, uint32_t pupd, int alt_func)
{
    MICRO_ASSERT(pin.isValid());
    MICRO_ASSERT(alt_func < 16);

    GPIO_TypeDef *port_reg = get_port_register(pin.port);

    uint32_t mask;
    uint32_t clear_mask;

    get_masks(mode, pin.num, 2, mask, clear_mask);
    port_reg->MODER = (port_reg->MODER & clear_mask) | mask;

    get_masks(otype, pin.num, 1, mask, clear_mask);
    port_reg->OTYPER = (port_reg->OTYPER & clear_mask) | mask;

    get_masks(ospeed, pin.num, 2, mask, clear_mask);
    port_reg->OSPEEDR = (port_reg->OSPEEDR & clear_mask) | mask;

    get_masks(pupd, pin.num, 2, mask, clear_mask);
    port_reg->PUPDR = (port_reg->PUPDR & clear_mask) | mask;

    get_masks(alt_func, pin.num % 8, 4, mask, clear_mask);
    const uint8_t afr_index = pin.num / 8;
    port_reg->AFR[afr_index] = (port_reg->AFR[afr_index] & clear_mask) | mask;
}

} // namespace


void gpio::configureOutput(Pin pin, OutputMode mode, OutputSpeed speed, PullMode pull_mode)
{
    const uint32_t mode_mask = 0b01;
    const uint32_t otype = mode == OutputMode::PushPull ? 0 : 1;

    uint32_t ospeed = 0;
    switch (speed)
    {
    case OutputSpeed::Low: ospeed = 0b00; break;
    case OutputSpeed::Medium: ospeed = 0b01; break;
    case OutputSpeed::High: ospeed = 0b10; break;
    case OutputSpeed::Max: ospeed = 0b11; break;
    default: MICRO_ASSERT(0); break;
    }

    uint32_t pupd = 0;
    switch (pull_mode)
    {
    case PullMode::None: pupd = 0b00; break;
    case PullMode::Down: pupd = 0b10; break;
    case PullMode::Up: pupd = 0b01; break;
    default: MICRO_ASSERT(0); break;
    }

    configure(pin, mode_mask, otype, ospeed, pupd, 0);
}

void gpio::configureInput(Pin pin, PullMode pull_mode)
{
    const uint32_t mode_mask = 0b00;
    const uint32_t otype = 0;
    const uint32_t ospeed = 0;

    uint32_t pupd = 0;
    switch (pull_mode)
    {
    case PullMode::None: pupd = 0b00; break;
    case PullMode::Down: pupd = 0b10; break;
    case PullMode::Up: pupd = 0b01; break;
    default: MICRO_ASSERT(0); break;
    }

    configure(pin, mode_mask, otype, ospeed, pupd, 0);
}

void gpio::configureAlternate(Pin pin, int alt_func, OutputMode mode, OutputSpeed speed, PullMode pull_mode)
{
    const uint32_t mode_mask = 0b10;
    const uint32_t otype = mode == OutputMode::PushPull ? 0 : 1;

    uint32_t ospeed = 0;
    switch (speed)
    {
    case OutputSpeed::Low: ospeed = 0b00; break;
    case OutputSpeed::Medium: ospeed = 0b01; break;
    case OutputSpeed::High: ospeed = 0b10; break;
    case OutputSpeed::Max: ospeed = 0b11; break;
    default: MICRO_ASSERT(0); break;
    }

    uint32_t pupd = 0;
    switch (pull_mode)
    {
    case PullMode::None: pupd = 0b00; break;
    case PullMode::Down: pupd = 0b10; break;
    case PullMode::Up: pupd = 0b01; break;
    default: MICRO_ASSERT(0); break;
    }

    configure(pin, mode_mask, otype, ospeed, pupd, alt_func);
}

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
