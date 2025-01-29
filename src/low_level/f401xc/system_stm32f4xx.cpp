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
        FLASH->ACR |= FLASH_ACR_PRFTEN;   // Enable prefetch buffer
        FLASH->ACR &= ~FLASH_ACR_LATENCY; // Clear latency settings
        FLASH->ACR
            |= FLASH_ACR_LATENCY_2WS; // Set 2 wait states for 84 MHz <button class="citation-flag" data-index="6">

        // Step 3: Configure the PLL
        RCC->PLLCFGR = 0;                                        // Reset PLL configuration
        RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;                  // Set PLL source to HSE
        RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_2 | RCC_PLLCFGR_PLLM_0; // Divide HSE by 8 (assuming 8 MHz crystal)
        RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_4 | RCC_PLLCFGR_PLLN_3 | RCC_PLLCFGR_PLLN_2 | RCC_PLLCFGR_PLLN_1
            | RCC_PLLCFGR_PLLN_0;           // Multiply by 168
        RCC->PLLCFGR |= RCC_PLLCFGR_PLLP_1; // Divide by 2 to get 84 MHz

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
        RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  // AHB prescaler: SYSCLK / 1
        RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // APB1 prescaler: HCLK / 2
        RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; // APB2 prescaler: HCLK / 1

        // Update global system frequency variable
        glob::SYS_FREQUENCY = 84'000'000;
    }
}
