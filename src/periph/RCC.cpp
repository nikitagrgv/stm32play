#include "RCC.h"

#include "DeviceCMSIS.h"

namespace
{

#ifdef STM32F103
void get_masks(uint32_t periph, uint32_t &apb1_mask, uint32_t &apb2_mask)
{
    using namespace rcc;
    if (periph & GPIO_A)
    {
        apb2_mask |= RCC_APB2ENR_IOPAEN;
    }
    if (periph & GPIO_B)
    {
        apb2_mask |= RCC_APB2ENR_IOPBEN;
    }
    if (periph & GPIO_C)
    {
        apb2_mask |= RCC_APB2ENR_IOPCEN;
    }
    if (periph & SYSCFG_OR_AFIO)
    {
        apb2_mask |= RCC_APB2ENR_AFIOEN;
    }
    if (periph & USART_1)
    {
        apb2_mask |= RCC_APB2ENR_USART1EN;
    }
    if (periph & TIM_2)
    {
        apb1_mask |= RCC_APB1ENR_TIM2EN;
    }
}
#elifdef STM32F401
void get_masks(uint32_t periph, uint32_t &apb1_mask, uint32_t &apb2_mask)
{
    using namespace rcc;
    if (periph & GPIO_A)
    {}
    if (periph & GPIO_B)
    {}
    if (periph & GPIO_C)
    {}
    if (periph & AFIO)
    {}
    if (periph & USART_1)
    {}
    if (periph & TIM_2)
    {}
}
#else
    #error
#endif

} // namespace

void rcc::enableClocks(uint32_t periphs)
{
    uint32_t apb1_mask = 0;
    uint32_t apb2_mask = 0;
    get_masks(periphs, apb1_mask, apb2_mask);

    RCC->APB1ENR |= apb1_mask;
    RCC->APB2ENR |= apb2_mask;
}

void rcc::disableClocks(uint32_t periphs)
{
    uint32_t apb1_mask = 0;
    uint32_t apb2_mask = 0;
    get_masks(periphs, apb1_mask, apb2_mask);

    RCC->APB1ENR &= ~apb1_mask;
    RCC->APB2ENR &= ~apb2_mask;
}

void rcc::setClocksEnabled(uint32_t periphs, bool enabled)
{
    if (enabled)
    {
        enableClocks(periphs);
    }
    else
    {
        disableClocks(periphs);
    }
}
