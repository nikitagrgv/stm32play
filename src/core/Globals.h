#pragma once

#include <cstdint>

namespace glob
{

constexpr uint32_t SYS_FREQUENCY = 8'000'000;

extern volatile uint32_t total_msec;

} // namespace glob
