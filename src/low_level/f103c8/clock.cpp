#include "low_level/clock.h"

#include "DeviceCMSIS.h"

#define HSE_VALUE ((uint32_t)8000000U) // Default value of the External oscillator in Hz
#define HSI_VALUE ((uint32_t)8000000U) // Value of the Internal oscillator in Hz

const uint8_t AHBPrescTable[16U] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8U] = {0, 0, 0, 0, 1, 2, 3, 4};

uint32_t calcSystemCoreClock()
{
    uint32_t ret = 0U;
    uint32_t tmp = 0U;

    tmp = RCC->CFGR & RCC_CFGR_SWS;

    switch (tmp)
    {
    case 0x00U:
        // HSI used as system clock
        ret = HSI_VALUE;
        break;
    case 0x04U:
        // HSE used as system clock
        ret = HSE_VALUE;
        break;
    case 0x08U:
    {
        // PLL used as system clock source
        uint32_t pllsource = 0U;
        uint32_t pllmull = 0U;
        pllmull = RCC->CFGR & RCC_CFGR_PLLMULL;
        pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;

        pllmull = (pllmull >> 18U) + 2U;

        if (pllsource == 0x00U)
        {
            // HSI oscillator clock divided by 2 selected as PLL clock entry
            ret = (HSI_VALUE >> 1U) * pllmull;
        }
        else
        {

            // HSE selected as PLL clock entry
            if ((RCC->CFGR & RCC_CFGR_PLLXTPRE) != (uint32_t)RESET)
            {
                // HSE oscillator clock divided by 2
                ret = (HSE_VALUE >> 1U) * pllmull;
            }
            else
            {
                ret = HSE_VALUE * pllmull;
            }
        }

        break;
    }
    default: ret = HSI_VALUE; break;
    }

    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4U)];
    ret >>= tmp;
    return ret;
}
