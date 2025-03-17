#pragma once

#include <cstdint>

namespace glob
{

extern volatile uint32_t SYSTEM_CORE_CLOCK;
extern volatile uint32_t APB1_PERIPH_CLOCK;
extern volatile uint32_t APB1_TIMER_CLOCK;
extern volatile uint32_t APB2_PERIPH_CLOCK;
extern volatile uint32_t APB2_TIMER_CLOCK;

extern volatile uint32_t total_msec;

} // namespace glob
