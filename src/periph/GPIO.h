#pragma once

#include "PeriphBase.h"
#include "core/MicroAssert.h"

namespace gpio
{

enum class PinMode
{
    InputFloating = 0b0100,
    InputPullUpOrDown = 0b1000,
    GeneralPushPull50MHz = 0b0011,
    GeneralOpenDrain50MHz = 0b0111,
    AlternatePushPull50MHz = 0b1011,
    AlternateOpenDrain50MHz = 0b1111,
};

enum class PullUpOrDownMode
{
    Down = 0,
    Up = 1,
};

void setPinMode(Pin pin, PinMode mode);
void disablePin(Pin pin);

bool getPinInput(Pin pin);
void setPinOutput(Pin pin, bool value);

void setPinPullUpOrDown(Pin pin, PullUpOrDownMode mode);

} // namespace gpio
