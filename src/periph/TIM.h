#pragma once

#include <cstdint>
#include <stm32f1xx.h>

namespace tim
{

constexpr uint32_t MAX_PRESCALER = 0xFFFF;
constexpr uint32_t MAX_RELOAD_VALUE = 0xFFFF;

enum SetupFlags : uint32_t
{
    SINGLE_SHOT = 1 << 0,
    ENABLE_UPDATE_INTERRUPT = 1 << 1,
};

void setupTimer(TIM_TypeDef *tim, uint32_t frequency, uint32_t reload_value, uint32_t setup_flags = 0);

void restartTimer(TIM_TypeDef *tim);
void stopTimer(TIM_TypeDef *tim);

uint32_t getTimerValue(TIM_TypeDef *tim);

} // namespace tim