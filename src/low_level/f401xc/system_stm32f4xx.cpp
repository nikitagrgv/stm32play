#include "DeviceCMSIS.h"
#include "core/Globals.h"

extern "C"
{
    void SystemInit(void)
    {
        // FPU enable
        SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); // set CP10 and CP11 Full Access

        // Step 1: Enable HSE (High-Speed External) clock
        RCC->CR |= RCC_CR_HSEON; // Enable HSE
        while (!(RCC->CR & RCC_CR_HSERDY))
            ; // Wait until HSE is ready

        // Step 2: Configure Flash prefetch and latency
        FLASH->ACR |= FLASH_ACR_PRFTBE;    // Enable prefetch buffesr
        FLASH->ACR &= ~FLASH_ACR_LATENCY;  // Clear latency settings
        FLASH->ACR |= FLASH_ACR_LATENCY_2; // Set 2 wait states for 72 MHz

        // Step 3: Configure the PLL
        RCC->CFGR &= ~RCC_CFGR_PLLSRC;  // Clear PLL source
        RCC->CFGR |= RCC_CFGR_PLLSRC;   // Set PLL source to HSE
        RCC->CFGR &= ~RCC_CFGR_PLLMULL; // Clear PLL multiplier bits
        RCC->CFGR |= RCC_CFGR_PLLMULL9; // Set PLL multiplier to 9 (8 MHz × 9 = 72 MHz)

        // Step 4: Enable the PLL
        RCC->CR |= RCC_CR_PLLON; // Enable PLL
        while (!(RCC->CR & RCC_CR_PLLRDY))
            ; // Wait until PLL is ready

        // Step 5: Select the PLL as the system clock source
        RCC->CFGR &= ~RCC_CFGR_SW;    // Clear system clock switch bits
        RCC->CFGR |= RCC_CFGR_SW_PLL; // Set system clock switch to PLL
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
            ; // Wait until PLL is the system clock source

        // Step 6: Configure the AHB, APB1, and APB2 prescalers
        RCC->CFGR &= ~RCC_CFGR_HPRE;  // AHB prescaler: SYSCLK / 1
        RCC->CFGR &= ~RCC_CFGR_PPRE1; // APB1 prescaler: HCLK / 2
        RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
        RCC->CFGR &= ~RCC_CFGR_PPRE2; // APB2 prescaler: HCLK / 1

        glob::SYS_FREQUENCY = 72'000'000;
    }
}
