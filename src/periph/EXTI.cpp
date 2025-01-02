#include "EXTI.h"

#include "core/Base.h"
#include "core/MicroAssert.h"

#include <stm32f1xx.h>

namespace
{

FORCE_INLINE constexpr uint32_t get_mask(int port_index, int pos)
{
    MICRO_ASSERT(pos >= 0 && pos < 8);
    const int bit_pos = pos * 4;
    return (uint32_t)port_index << bit_pos;
}

FORCE_INLINE constexpr uint32_t get_clear_mask(int pos)
{
    MICRO_ASSERT(pos >= 0 && pos < 8);
    return ~(0b1111UL << (pos * 4));
}

FORCE_INLINE constexpr int get_port_index(GPIOPort port)
{
    return int(port);
}

} // namespace

void exti::setupEXTI(GPIOPort port, int pin, TriggerMode mode, uint32_t flags)
{
    MICRO_ASSERT(pin >= 0 && pin < 16);

    const int port_index = get_port_index(port);
    const int cr_reg_index = pin / 4;
    const int cr_reg_exti_pos = pin % 4;

    const uint32_t clear_mask = get_clear_mask(cr_reg_exti_pos);
    const uint32_t mask = get_mask(port_index, cr_reg_exti_pos);
    auto &reg = AFIO->EXTICR[cr_reg_index];
    reg = (reg & clear_mask) | mask;

    const uint32_t pin_bit_mask = 1UL << pin;

    // Setup edges
    if ((uint32_t)mode & (uint32_t)TriggerMode::FallingEdges)
    {
        EXTI->FTSR |= pin_bit_mask;
    }
    else
    {
        EXTI->FTSR &= ~pin_bit_mask;
    }

    if ((uint32_t)mode & (uint32_t)TriggerMode::RisingEdges)
    {
        EXTI->RTSR |= pin_bit_mask;
    }
    else
    {
        EXTI->RTSR &= ~pin_bit_mask;
    }

    // Setup interrupts
    if (flags & ENABLE_INTERRUPT)
    {
        EXTI->IMR |= pin_bit_mask;
    }
    else
    {
        EXTI->IMR &= ~pin_bit_mask;
    }
}

InterruptType exti::getInterruptType(int pin)
{
    MICRO_ASSERT(pin >= 0 && pin < 16);
    const int exti_line = pin / 4;
    const int type = (int)InterruptType::EXTI0IRQ + exti_line;
    return (InterruptType)(type);
}
