#include "DeviceCMSIS.h"

namespace
{

void setup_clock()
{
    // Clear previous state
    RCC->CFGR = 0;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
    {}

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
    RCC->CFGR |= RCC_CFGR_SW_PLL | RCC_CFGR_PPRE1_DIV2;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    {
        // Wait until the PLL is used as the system clock source
    }
}

} // namespace

extern "C"
{
    void SystemInit(void)
    {
        // set CP10 and CP11 Full Access
        SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));

        setup_clock();
    }
}