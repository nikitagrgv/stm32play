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


#define HSE_VALUE ((uint32_t)25000000) /*!< Default value of the External oscillator in Hz */

#define HSI_VALUE ((uint32_t)16000000) /*!< Value of the Internal oscillator in Hz*/


uint32_t SystemCoreClock = 16000000;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};


int pll_is_hse = 0;

void SystemCoreClockUpdate(void)
{
    pll_is_hse = 0;

    uint32_t tmp = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;

    tmp = RCC->CFGR & RCC_CFGR_SWS;

    switch (tmp)
    {
    case 0x00: /* HSI used as system clock source */ SystemCoreClock = HSI_VALUE; break;
    case 0x04: /* HSE used as system clock source */ SystemCoreClock = HSE_VALUE; break;
    case 0x08: /* PLL used as system clock source */

        /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
           SYSCLK = PLL_VCO / PLL_P
           */
        pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
        pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;

        if (pllsource != 0)
        {
            pll_is_hse = 1;
            /* HSE used as PLL clock source */
            pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
        }
        else
        {
            /* HSI used as PLL clock source */
            pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
        }

        pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) + 1) * 2;
        SystemCoreClock = pllvco / pllp;
        break;
    default: SystemCoreClock = HSI_VALUE; break;
    }
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
    SystemCoreClock >>= tmp;
}


int main(void)
{
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */


    RCC->CFGR = 0;
    for (volatile uint32_t a = 0xfffff; a > 0; --a)
    {}

    {
        // Step 1: Enable HSE (High-Speed External) oscillator
        RCC->CR |= RCC_CR_HSEON;
        while (!(RCC->CR & RCC_CR_HSERDY))
        {
            // Wait until HSE is ready
        }

        // Step 2: Select HSI as the system clock source to allow reconfiguration
        RCC->CFGR &= ~RCC_CFGR_SW; // Clear SW bits
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
        {
            // Wait until HSI is used as the system clock source
        }

        // Step 3: Disable the PLL
        RCC->CR &= ~RCC_CR_PLLON;
        while (RCC->CR & RCC_CR_PLLRDY)
        {
            // Wait until PLL is fully stopped
        }

        // Step 4: Configure the PLL
        // PLL_M = 25, PLL_N = 336, PLL_P = 4, PLL_Q = 7
        RCC->PLLCFGR = (25 << RCC_PLLCFGR_PLLM_Pos) | // PLL_M
            (336 << RCC_PLLCFGR_PLLN_Pos) |           // PLL_N
            (1 << RCC_PLLCFGR_PLLP_Pos) |             // PLL_P (00: PLLP = 2; 01: PLLP = 4; 10: PLLP = 6; 11: PLLP = 8)
            (RCC_PLLCFGR_PLLSRC_HSE) |                // PLL source
            (7 << RCC_PLLCFGR_PLLQ_Pos);              // PLL_Q

        // Step 5: Enable the PLL
        RCC->CR |= RCC_CR_PLLON;
        while (!(RCC->CR & RCC_CR_PLLRDY))
        {
            // Wait until PLL is ready
        }

        // Step 6: Configure Flash prefetch, Instruction cache, Data cache, and wait state
        FLASH->ACR = FLASH_ACR_PRFTEN | // Enable prefetch
            FLASH_ACR_ICEN |            // Instruction cache enable
            FLASH_ACR_DCEN |            // Data cache enable
            FLASH_ACR_LATENCY_2WS;      // Flash latency

        // Step 7: Select the PLL as the system clock source
        RCC->CFGR |= RCC_CFGR_SW_PLL;
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        {
            // Wait until the PLL is used as the system clock source
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
    // SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;


    // Main loop
    while (1)
    {
        gpio::setPinOutput(led_pin, pll_is_hse);
        // Toggle an LED or perform some action every 1 second
        // For example:
        // GPIO_ToggleBits(GPIOD, GPIO_Pin_12);  // Assuming an LED is connected to PD12

        // delay_ms(1000);
    }
}