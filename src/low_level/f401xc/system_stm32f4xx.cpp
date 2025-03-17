#include "DeviceCMSIS.h"

namespace
{

void setup_clock()
{
    // Clear the previous state. Select HSI as the system clock source to allow reconfiguration.
    RCC->CFGR = 0;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
    {}

    // Disable the PLL
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY)
    {}

    // Enable HSE (High-Speed External) oscillator
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY))
    {}

    // Configure the PLL
    // PLL_M = 25, PLL_N = 336, PLL_P = 4, PLL_Q = 7
    RCC->PLLCFGR = (25 << RCC_PLLCFGR_PLLM_Pos) | // PLL_M
        (336 << RCC_PLLCFGR_PLLN_Pos) |           // PLL_N
        (1 << RCC_PLLCFGR_PLLP_Pos) |             // PLL_P (00: PLLP = 2; 01: PLLP = 4; 10: PLLP = 6; 11: PLLP = 8)
        (RCC_PLLCFGR_PLLSRC_HSE) |                // PLL source
        (7 << RCC_PLLCFGR_PLLQ_Pos);              // PLL_Q

    // Enable the PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY))
    {}

    // Configure Flash prefetch, Instruction cache, Data cache, and wait state
    FLASH->ACR = FLASH_ACR_PRFTEN | // Enable prefetch
        FLASH_ACR_ICEN |            // Instruction cache enable
        FLASH_ACR_DCEN |            // Data cache enable
        FLASH_ACR_LATENCY_2WS;      // Flash latency

    // Select the PLL as the system clock source
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