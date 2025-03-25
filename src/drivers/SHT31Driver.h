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
    ~SHT31Driver();

    ErrorCode run(float &temperature, float &humidity);

private:
    I2C i2c_;
};