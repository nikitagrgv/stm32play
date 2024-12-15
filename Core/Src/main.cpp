#include <cassert>
#include <cmath>
#include <cstdarg>
#include <cstdio>
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
    size_t last_msec = 0;
    void SysTick_Handler()
    {
        ++total_msec;
    }

    void USART1_IRQHandler(void)
    {
        if (USART1->SR & USART_SR_RXNE)
        {
            switch_led();
            volatile uint8_t data = USART1->DR;
            volatile char c = data;
        }
    }
}

enum class GPIOMode
{
    FloatingInput = 0b0100,
    GeneralPushPull50MHz = 0b0011,
    GeneralOpenDrain50MHz = 0b0111,
    AlternatePushPull50MHz = 0b1011,
    AlternateOpenDrain50MHz = 0b1111,
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
    const uint32_t clear_mask = getGPIOClearMask(pos);
    const uint32_t mask = getGPIOMask(mode, pos);
    reg = (reg & clear_mask) | mask;
}

void printSync(const char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);

    constexpr int BUFFER_SIZE = 100;
    char buffer[BUFFER_SIZE];

    vsnprintf(buffer, BUFFER_SIZE, fmt, va);

    const char *p = buffer;
    while (*p)
    {
        while (!(USART1->SR & USART_SR_TXE))
        {}
        USART1->DR = *p++;
    }

    va_end(va);
}

int main()
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN
        | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;

    // C13 open drain
    setPinMode(GPIOC, 13, GPIOMode::GeneralOpenDrain50MHz);

    setPinMode(GPIOA, 9, GPIOMode::AlternatePushPull50MHz); // USART1 TX
    setPinMode(GPIOA, 10, GPIOMode::FloatingInput);         // USART1 RX

    // SysTick
    SysTick->LOAD = 8'000 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    // -------------------------------------------------------------------------
    // USART1 - for Modbus. 9600 baudrate
    // -------------------------------------------------------------------------
    USART1->BRR = 833; // USARTDIV = 8e6 / 9600 = 833
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    NVIC_EnableIRQ(USART1_IRQn);
    __enable_irq(); // enable interrupts

    last_msec = total_msec;
    while (true)
    {
        if (total_msec - last_msec >= 1000)
        {
            last_msec = total_msec;
            switch_led();
            printSync("Total msec = %d\n", total_msec);
        }
    }
}
