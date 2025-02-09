#include "low_level/clock.h"

#include "DeviceCMSIS.h"

#define HSE_VALUE ((uint32_t)25000000) // Default value of the External oscillator in Hz
#define HSI_VALUE ((uint32_t)16000000) // Value of the Internal oscillator in Hz

const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};

uint32_t calcSystemCoreClock()
{
    uint32_t ret = 0;
    uint32_t tmp = 0;

    tmp = RCC->CFGR & RCC_CFGR_SWS;

    switch (tmp)
    {
    case 0x00:
        // HSI used as system clock source
            ret = HSI_VALUE;
        break;
    case 0x04:
        // HSE used as system clock source
            ret = HSE_VALUE;
        break;
    case 0x08:
    {
        // PLL used as system clock source
        uint32_t pllm = 2;
        uint32_t pllsource = 0;
        uint32_t pllp = 2;
        uint32_t pllvco = 0;

        // PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
        // SYSCLK = PLL_VCO / PLL_P
        pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
        pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;

        if (pllsource != 0)
        {
            // HSE used as PLL clock source
            pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
        }
        else
        {
            // HSI used as PLL clock source
            pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
        }

        pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) + 1) * 2;
        ret = pllvco / pllp;
        break;
    }
    default: ret = HSI_VALUE; break;
    }

    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
    ret >>= tmp;
    return ret;
}
