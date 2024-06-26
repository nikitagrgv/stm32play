#include <cassert>
#include <cmath>
#include <stm32f103xb.h>

bool led_state = false;
void switch_led()
{
    const uint32_t val = led_state ? GPIO_BSRR_BR13 : GPIO_BSRR_BS13;
    led_state = !led_state;
    GPIOC->BSRR = val;
}

extern "C"
{
    uint32_t total_msec = 0;
    void SysTick_Handler()
    {
        ++total_msec;
    }
}

enum class GPIOMode
{
    FloatingInput = 0b0100,
    GeneralOpenDrain50MHz = 0b0101,
};

constexpr uint32_t getGPIOMask(GPIOMode mode, int pos)
{
    assert(pos < 8);
    const int bit_pos = pos * 4;
    return (uint32_t)mode << bit_pos;
}

constexpr uint32_t getGPIOClearMask(int pos)
{
    assert(pos < 8);
    return ~(0b1111UL << (pos * 4));
}

void setPinMode(GPIO_TypeDef *port, int pin, GPIOMode mode)
{
    const int is_high = pin >= 8;
    const int pos = pin % 8;
    auto &reg = is_high ? port->CRH : port->CRL;
    const uint32_t mask = getGPIOMask(mode, pos);
    const uint32_t clear_mask = getGPIOClearMask(pos);
    reg = (reg & ~clear_mask) | mask;
}

int main()
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN
        | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;

    // C13 open drain
    setPinMode(GPIOC, 13, GPIOMode::GeneralOpenDrain50MHz);

    // SysTick
    SysTick->LOAD = 8'000 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    size_t last_msec = total_msec;
    while (true)
    {
        if (total_msec - last_msec >= 500)
        {
            last_msec = total_msec;
            switch_led();
        }
    }
}
