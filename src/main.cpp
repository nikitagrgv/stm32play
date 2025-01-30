// #include "DeviceCMSIS.h"
// #include "Print.h"
// #include "Sleep.h"
// #include "core/Globals.h"
// #include "periph/GPIO.h"
// #include "periph/IRQ.h"
// #include "periph/PeriphBase.h"
// #include "periph/SysTick.h"
//
// #include <memory>
//
//
// volatile bool go = true;
//
// int main()
// {
//     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
//     RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
//     RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
//     RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
//
//     constexpr Pin led_pin{GPIOPort::C, 13};
//     gpio::configureOutput(led_pin, gpio::OutputMode::OpenDrain, gpio::OutputSpeed::High);
//
//     irq::setHandler(InterruptType::SysTickIRQ, [](void *) {
//         go = true;
//     });
//
//
//
//
//     irq::enableInterrupts();
//
//     while (true)
//     {
//         if (!go)
//         {
//             continue;
//         }
//         go = false;
//
//         static bool a = false;
//         gpio::setPinOutput(led_pin, a);
//         a = !a;
//     }
// }


#include "periph/GPIO.h"
#include "stm32f4xx.h"

// Define the system clock frequency (in Hz)
#define SYSTEM_CLOCK_FREQ 84000000 // 84 MHz for STM32F401

static bool a = false;

constexpr Pin led_pin{GPIOPort::C, 13};

extern "C"
{
    void switchLed()
    {
        gpio::setPinOutput(led_pin, a);
        a = !a;
    }

    int count = 0;
    void SysTick_Handler()
    {
        ++count;
        if (count >= 500)
        {
            switchLed();
            count = 0;
        }
    }
}
int main(void)
{
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */



    RCC->CFGR = 0;
    for (volatile uint32_t a = 0xfffff; a > 0; --a)
    {}

    {
        // Enable HSE Oscillator
        RCC->CR |= RCC_CR_HSEON;
        while (!(RCC->CR & RCC_CR_HSERDY))
        {
            // Wait for HSE to become ready
        }

        // Configure PLL
        constexpr uint32_t g2 = (25 << RCC_PLLCFGR_PLLM_Pos) | // PLLM = 25
            (336 << RCC_PLLCFGR_PLLN_Pos) |                    // PLLN = 336
            (1 << RCC_PLLCFGR_PLLP_Pos) |                      // PLLP = 2 (0 corresponds to division by 2)
            RCC_PLLCFGR_PLLSRC_HSE;                            // PLL source is HSE
        RCC->PLLCFGR = g2;

        // Enable PLL
        RCC->CR |= RCC_CR_PLLON;
        while (!(RCC->CR & RCC_CR_PLLRDY))
        {
            // Wait for PLL to become ready
        }

        // Set PLL as system clock source
        RCC->CFGR = 0;
        RCC->CFGR |= RCC_CFGR_SW_PLL | (0b100 << RCC_CFGR_PPRE1_Pos);
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        {
            // Wait for PLL to be used as system clock
        }
    }

    // while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
    //     ;
    SystemCoreClockUpdate();


    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    gpio::configureOutput(led_pin, gpio::OutputMode::OpenDrain, gpio::OutputSpeed::High);

    constexpr uint32_t one_ms_ticks = SYSTEM_CLOCK_FREQ / 1000; // For 1 ms delay

    SysTick->CTRL = 0;
    SysTick->LOAD = one_ms_ticks - 1; // -1 because the counter counts down to zero
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;


    // Main loop
    while (1)
    {
        // Toggle an LED or perform some action every 1 second
        // For example:
        // GPIO_ToggleBits(GPIOD, GPIO_Pin_12);  // Assuming an LED is connected to PD12

        // delay_ms(1000);
    }
}