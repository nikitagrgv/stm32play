#pragma once

#include "PeriphBase.h"

namespace i2c
{

constexpr uint32_t DEFAULT_SPEED = 100'000;

void setupI2C(I2C i2c, uint32_t speed = DEFAULT_SPEED);

void masterTransmitBlocking(I2C i2c, uint8_t address, const uint8_t *buf, uint32_t num_bytes);
void masterReceiveBlocking(I2C i2c, uint8_t address, uint8_t *buf, uint32_t num_bytes);

void masterTransmitBlocking(I2C i2c, uint8_t address, uint8_t value);
void masterReceiveBlocking(I2C i2c, uint8_t address, uint8_t &value);

} // namespace i2c