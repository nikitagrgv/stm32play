#pragma once

#include <cstdint>
#include "DeviceCMSIS.h"

namespace utils
{

void sleepMsec(uint32_t msec);

void sleepMsec(TIM_TypeDef *timer, uint32_t msec);
void sleepUsec(TIM_TypeDef *timer, uint32_t usec);

} // namespace utils
