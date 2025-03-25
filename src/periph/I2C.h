#pragma once

#include "PeriphBase.h"

namespace i2c
{

constexpr uint32_t DEFAULT_SPEED = 100'000;

void setupI2C(I2C i2c, uint32_t speed = DEFAULT_SPEED);

bool masterTransmitBlocking(I2C i2c_reg, uint8_t address, const uint8_t *buf, uint32_t num_bytes);
bool masterReceiveBlocking(I2C i2c_reg, uint8_t address, uint8_t *buf, uint32_t num_bytes);

} // namespace i2c