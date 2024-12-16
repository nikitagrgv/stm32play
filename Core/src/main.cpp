#include "DataStream.h"
#include "Globals.h"
#include "Utils.h"

#include <cassert>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <stm32f103xb.h>

volatile bool led_state = false;
void switch_led()
{
    const uint32_t val = led_state ? GPIO_BSRR_BR13 : GPIO_BSRR_BS13;
    led_state = !led_state;
    GPIOC->BSRR = val;
}


DataStream usart1_stream;

extern "C"
{
    void USART1_IRQHandler(void)
    {
        if (USART1->SR & USART_SR_RXNE)
        {
            switch_led();
            const uint8_t data = USART1->DR;
            usart1_stream.writeByte(data);
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

void printSync(const char *string)
{
    const char *p = string;
    while (*p)
    {
        while (!(USART1->SR & USART_SR_TXE))
        {}
        USART1->DR = *p++;
    }
}


void printSyncFmt(const char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);

    constexpr int BUFFER_SIZE = 1024;
    char buffer[BUFFER_SIZE];

    vsnprintf(buffer, BUFFER_SIZE, fmt, va);

    printSync(buffer);

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

    // USART1
    constexpr int baudrate = 9600;
    constexpr int brr_value = 8'000'000 / baudrate; // USARTDIV
    USART1->BRR = brr_value;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    NVIC_EnableIRQ(USART1_IRQn);
    __enable_irq(); // enable interrupts

    constexpr int BUFFER_SIZE = 1024;
    uint8_t buffer[BUFFER_SIZE + 1];
    while (true)
    {
        constexpr int THROTTLING_MSEC = 1000;
        const int num_read = usart1_stream.readDataThrottling(buffer, BUFFER_SIZE, THROTTLING_MSEC);

        if (num_read == 0)
        {
            continue;
        }

        buffer[num_read] = 0;

        printSyncFmt((char *)buffer);
        printSync("\n");
    }
}
