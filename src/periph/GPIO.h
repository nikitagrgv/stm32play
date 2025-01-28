#pragma once

#include "PeriphBase.h"
#include "core/MicroAssert.h"

namespace gpio
{

enum class PullMode
{
    None,
    Down,
    Up
};

enum class OutputMode
{
    PushPull,
    OpenDrain,
};

enum class OutputSpeed
{
    Low,
    Medium,
    High,
    Max
};

void configureOutput(Pin pin, OutputMode mode, OutputSpeed speed, PullMode pull_mode = PullMode::None);
void configureInput(Pin pin, PullMode pull_mode = PullMode::None);

void disablePin(Pin pin);

bool getPinInput(Pin pin);
void setPinOutput(Pin pin, bool value);

} // namespace gpio
