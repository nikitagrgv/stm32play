#pragma once

#include <cstdint>

namespace systick
{

enum SetupFlags : uint32_t
{
    ENABLE_INTERRUPT = 1 << 0,
};

void setupTimer(uint32_t frequency, uint32_t setup_flags = 0);
void restartTimer();

} // namespace systick