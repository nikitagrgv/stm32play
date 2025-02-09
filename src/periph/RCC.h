#pragma once

#include <cstdint>

namespace rcc
{

enum : uint32_t
{
    GPIO_A = 1 << 0,
    GPIO_B = 1 << 1,
    GPIO_C = 1 << 2,

    SYSCFG_AFIO = 1 << 3,

    USART_1 = 1 << 4,

    TIM_2 = 1 << 5,
};

void enableClocks(uint32_t periphs);
void disableClocks(uint32_t periphs);
void setClocksEnabled(uint32_t periphs, bool enabled);

} // namespace rcc
