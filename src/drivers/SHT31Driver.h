#pragma once

#include "periph/PeriphBase.h"

class SHT31Driver
{
public:
    enum class ErrorCode
    {
        Success,
        Timeout,
        InvalidChecksum,
    };

    explicit SHT31Driver(I2C i2c);

    ErrorCode run(float &temperature, float &humidity);

private:
    static constexpr int TIMEOUT_MS = 100;

private:
    I2C i2c_;
};