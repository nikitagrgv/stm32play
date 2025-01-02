#pragma once

#include "PeriphBase.h"

#include <cstdint>

namespace exti
{

enum class TriggerMode
{
    FallingEdges = 1 << 0,
    RisingEdges = 1 << 1,
    BothEdges = FallingEdges | RisingEdges,
};

enum SetupFlags : uint32_t
{
    ENABLE_INTERRUPT = 1 << 0,
};

void setupEXTI(GPIOPort port, int pin, TriggerMode mode, uint32_t flags);

} // namespace exti
