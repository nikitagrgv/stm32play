#pragma once

#include "PeriphBase.h"

namespace i2c
{

constexpr uint32_t DEFAULT_SPEED = 100'000;

void setupI2C(I2C i2c, uint32_t speed);

} // namespace i2c