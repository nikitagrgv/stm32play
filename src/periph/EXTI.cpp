#include "EXTI.h"

#include "DeviceCMSIS.h"
#include "core/Base.h"
#include "core/MicroAssert.h"

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

void exti::setupEXTI(Pin pin, TriggerMode mode, uint32_t flags)
{
    MICRO_ASSERT(pin.isValid());

    disableEXTI(pin.num);

    const int port_index = get_port_index(pin.port);
    const int cr_reg_index = pin.num / 4;
    const int cr_reg_exti_pos = pin.num % 4;

    const uint32_t clear_mask = get_clear_mask(cr_reg_exti_pos);
    const uint32_t mask = get_mask(port_index, cr_reg_exti_pos);

#ifdef STM32F103
    auto &reg = AFIO->EXTICR[cr_reg_index];
#elifdef STM32F401
    auto &reg = SYSCFG->EXTICR[cr_reg_index];
#else
    #error
#endif

    reg = (reg & clear_mask) | mask;

    const uint32_t pin_bit_mask = 1UL << pin.num;

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

void exti::disableEXTI(int line)
{
    const uint32_t line_bit_mask = 1UL << line;
    const uint32_t line_bit_clear_mask = ~line_bit_mask;

    EXTI->FTSR &= line_bit_clear_mask;
    EXTI->RTSR &= line_bit_clear_mask;
    EXTI->SWIER &= line_bit_clear_mask;
    EXTI->IMR &= line_bit_clear_mask;
    EXTI->EMR &= line_bit_clear_mask;
    EXTI->PR = line_bit_mask; // NOTE: write 1 to clear
}

bool exti::checkPendingAndClear(int line)
{
    const uint32_t line_bit_mask = 1UL << line;
    if (EXTI->PR & line_bit_mask)
    {
        EXTI->PR = line_bit_mask;
        return true;
    }
    return false;
}

InterruptType exti::getInterruptType(int pin)
{
    MICRO_ASSERT(pin >= 0 && pin < 16);

    if (pin < 5)
    {
        const int type = (int)InterruptType::EXTI0IRQ + pin;
        return (InterruptType)(type);
    }
    if (pin < 10)
    {
        return InterruptType::EXTI5_9_IRQn;
    }
    return InterruptType::EXTI10_15_IRQn;
}
