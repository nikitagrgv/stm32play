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

void setPinMode(GPIOPort port, int pin, PinMode mode);
void disablePin(GPIOPort port, int pin);

bool getPinInput(GPIOPort port, int pin);
void setPinOutput(GPIOPort port, int pin, bool value);

void setPinPullUpOrDown(GPIOPort port, int pin, PullUpOrDownMode mode);

} // namespace gpio
