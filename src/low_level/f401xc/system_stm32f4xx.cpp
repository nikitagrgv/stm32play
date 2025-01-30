#include "DeviceCMSIS.h"
#include "core/Globals.h"

extern "C"
{
    // void SystemInit(void)
    // {
    // // FPU enable
    // SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));
    //
    // // Enable HSE Oscillator
    // RCC->CR |= RCC_CR_HSEON;
    // while (!(RCC->CR & RCC_CR_HSERDY))
    // {
    //     // Wait for HSE to become ready
    // }
    //
    // // Configure PLL
    // RCC->PLLCFGR = (25 << RCC_PLLCFGR_PLLM_Pos) | // PLLM = 25
    //     (336 << RCC_PLLCFGR_PLLN_Pos) |           // PLLN = 336
    //     (1 << RCC_PLLCFGR_PLLP_Pos) |             // PLLP = 2 (0 corresponds to division by 2)
    //     RCC_PLLCFGR_PLLSRC_HSE;                   // PLL source is HSE
    //
    // // Enable PLL
    // RCC->CR |= RCC_CR_PLLON;
    // while (!(RCC->CR & RCC_CR_PLLRDY))
    // {
    //     // Wait for PLL to become ready
    // }
    //
    // // Set PLL as system clock source
    // RCC->CFGR = 0;
    // RCC->CFGR |= RCC_CFGR_SW_PLL | (0b100 << RCC_CFGR_PPRE1_Pos);
    // while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    // {
    //     // Wait for PLL to be used as system clock
    // }
    // // Update global system frequency variable
    // glob::SYS_FREQUENCY = 84'000'000;
    // }

#define HSE_VALUE ((uint32_t)25000000) /*!< Default value of the External oscillator in Hz */

#define HSI_VALUE ((uint32_t)16000000) /*!< Value of the Internal oscillator in Hz*/


    uint32_t SystemCoreClock = 16000000;
    const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
    const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};

    void SystemInit(void)
    {
        return;
        SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */


        if((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL)
        {
            RCC->CFGR &= (uint32_t) (~RCC_CFGR_SW);
            while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
        }




        // FPU enable
        SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));

        // Enable HSE Oscillator
        RCC->CR |= RCC_CR_HSEON;
        while (!(RCC->CR & RCC_CR_HSERDY))
        {
            // Wait for HSE to become ready
        }

        // Configure PLL
        RCC->PLLCFGR = (25 << RCC_PLLCFGR_PLLM_Pos) | // PLLM = 25
            (336 << RCC_PLLCFGR_PLLN_Pos) |           // PLLN = 336
            (1 << RCC_PLLCFGR_PLLP_Pos) |             // PLLP = 2 (0 corresponds to division by 2)
            RCC_PLLCFGR_PLLSRC_HSE;                   // PLL source is HSE

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
        // Update global system frequency variable
        glob::SYS_FREQUENCY = 84'000'000;



    }

    void SystemCoreClockUpdate(void)
    {
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
}